#include "localization_flow.h"
//#include "localization_config.h"
#include <pcl/common/transforms.h>
#include "glog/logging.h"
#include "global_definition/global_definition.h"

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include "tools/tic_toc.hpp"

LocalizationFlow::LocalizationFlow(ros::NodeHandle &nh):
    nh_(nh),
    rgb_d_sub_ptr_(std::make_shared<DualImgSubscriber>(nh_, "/d435i/color/image_raw", "/d435i/aligned_depth_to_color/image_raw", 20)),
    tf_listener_ptr_(std::make_shared<TFListener>()),
    tf_broadcast_ptr_(std::make_shared<TFBroadCaster>()),
    full_cloud_pub_ptr_(std::make_shared<CloudPublisher>(nh_, "/full_cloud", "/d435i_color_optical_frame", 20)),
    ball_cloud_pub_ptr_(std::make_shared<CloudPublisher>(nh_, "/ball_cloud", "/d435i_color_optical_frame", 20)),
    ball_vel_pub_ptr_(std::make_shared<ArrowPublisher>(nh_, "/ball_vel", "/t265_odom_frame", 20)),
    ball_estimator(nh),
    cv_vis(false),
    check_point_cloud(false),
    found_ball(false),
    full_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>()),
    ball_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()),
    time_run(std::make_shared<TicToc>("time per Run(): ", true)){

    // visualization opt
    nh_.getParam("cv_vis", cv_vis);
    nh_.getParam("check_point_cloud", check_point_cloud);

    // frame size
    nh_.getParam("camera2_color_width", FRAME_WIDTH);
    nh_.getParam("camera2_color_height", FRAME_HEIGHT);

    // distortion calib
    Eigen::Matrix3f K;
    /* align depth to color on
     * use align depth to color's K, which is the same as the K of color
     * different resolution, align_depth on/off  -->  different K */
    if(FRAME_WIDTH == 1280 && FRAME_HEIGHT == 720){
        K << 921.568359375, 0.0, 643.44873046875,
                0.0, 919.7422485351562, 379.1036071777344,
                0.0, 0.0, 1.0;
    }else if(FRAME_WIDTH == 848 && FRAME_HEIGHT == 480){
        K << 614.37890625, 0.0, 426.29913330078125,
                0.0, 613.1614990234375, 252.73573303222656,
                0.0, 0.0, 1.0;
    }else if(FRAME_WIDTH == 640 && FRAME_HEIGHT == 360){
        K << 460.7841796875, 0.0, 321.724365234375,
                0.0, 459.8711242675781, 189.5518035888672,
                0.0, 0.0, 1.0;
    }else{
        cerr << "-------This rgb resolution is not recommended! -----------" << endl;
    }
    K_inv = K.inverse();

    // valid z dis of depth img
    if(FRAME_WIDTH == 1280)
        clip_z_dis[0] = 0.25; // slightly smaller than min dis in datasheet
    else
        clip_z_dis[0] = 0.16;
    nh_.getParam("camera2_clip_distance", clip_z_dis[1]);
    if(clip_z_dis[1] < 0)
        clip_z_dis[1] = 30;
    // ball thresholding
    nh_.getParam("ball_thres_method", ball_thres_method);
    //   HSV
    nh_.getParam("H_MIN", H_MIN);
    nh_.getParam("H_MAX", H_MAX);
    nh_.getParam("S_MIN", S_MIN);
    nh_.getParam("S_MAX", S_MAX);
    nh_.getParam("V_MIN", V_MIN);
    nh_.getParam("V_MAX", V_MAX);
    //   RGB
    nh_.getParam("R_MIN", R_MIN);
    nh_.getParam("R_MAX", R_MAX);
    nh_.getParam("G_MIN", G_MIN);
    nh_.getParam("G_MAX", G_MAX);
    nh_.getParam("B_MIN", B_MIN);
    nh_.getParam("B_MAX", B_MAX);
    // obj filtering
    nh_.getParam("useMorphOps", useMorphOps);
    nh_.getParam("erode_diam", ERODE_DIAM);
    nh_.getParam("dilate_diam", DILATE_DIAM);
    nh_.getParam("min_obj_pix_diam", MIN_OBJ_PIX_DIAM);
    nh_.getParam("max_obj_percentaage", MAX_OBJ_PERCENTAGE);
    MIN_OBJECT_AREA = MIN_OBJ_PIX_DIAM * MIN_OBJ_PIX_DIAM;
    MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH * MAX_OBJ_PERCENTAGE / 100.;
    nh_.getParam("max_num_objs", MAX_NUM_OBJECTS); // for noise detection, too many obj -> too large noise

    //create slider bars for HSV filtering
    createTrackbars();
}



void LocalizationFlow::Run(){
//    time_run->tic();
    rgb_d_sub_ptr_->ParseData(rgb_d_buffer_);

    if(!rgb_d_buffer_.empty()){
        cur_rgbd_stamped = rgb_d_buffer_.back();
        rgb_d_buffer_.clear();
        cur_wheel_center = Eigen::Vector3f(0,0,0); // TODO: -----synchronize wheel center data with dual img msg------
        nh_.getParam("gnd_height", cur_wheel_center.z());

        try{
            tf_listener_ptr_->lookupTransform("/t265_odom_frame", "/d435i_color_optical_frame", cur_rgbd_stamped.time, cur_d435i_pos, cur_d435i_ori);
        }catch(tf2::LookupException &exc){ // wait until the camera boot node has fully started
            return;
        }catch(tf2::ConnectivityException &exc){
            return;
        }catch(tf2::ExtrapolationException &exc){
            return;
        }

        if(check_point_cloud){ // only generate a full cloud from rgb+d to check alignment with realsense point cloud
            // Publish the whole point cloud for checking with Realsense point cloud.
            // Remember to enable "pointcloud" filter to publish Realsense point cloud
            full_cloud_ptr->clear();
            GenerateFullPointCloud();
            full_cloud_pub_ptr_->Publish(full_cloud_ptr, cur_rgbd_stamped.time);
        }else{ // this is for real application
            if(cv_vis)
                updateParams();

            SegmentBall2D();

            // update ball pos & vel estimate
            if(found_ball){
                GetBallCloud();
                if(!ball_cloud_ptr->empty()){ // if detect ball in rgb but cloud is empty, keep the ball pos & vel the same as before to reduce noise
                    CalcBallCenter3D();
                    ball_estimator.addPos(cur_rgbd_stamped.time, ball_center, cur_wheel_center);

                    ball_cloud_pub_ptr_->Publish(ball_cloud_ptr, cur_rgbd_stamped.time);
                    // Publish ball tf wrt world frame, with orientation set to identity for convenience
                }
                tf_broadcast_ptr_->SendTransform("/t265_odom_frame", "/ball_real",
                                                 ball_center, Eigen::Quaternionf::Identity(), cur_rgbd_stamped.time);
                ball_vel_pub_ptr_->Publish(ball_center, ball_center + ball_estimator.getCurBallVel(), cur_rgbd_stamped.time);
            }else{
                ball_estimator.addLost(cur_rgbd_stamped.time, cur_wheel_center);
                if(!ball_estimator.isGiveup()){
                    ball_center = ball_estimator.getCurBallPos();
                    tf_broadcast_ptr_->SendTransform("/t265_odom_frame", "/ball_real",
                                                     ball_center, Eigen::Quaternionf::Identity(), cur_rgbd_stamped.time);
                    ball_vel_pub_ptr_->Publish(ball_center, ball_center + ball_estimator.getCurBallVel(), cur_rgbd_stamped.time);
                }
            }

            // TODO: calc control input

        }
    }
//    time_run->toc();
}

void LocalizationFlow::GenerateFullPointCloud(){
    cv_bridge::CvImageConstPtr rgb_ptr = cur_rgbd_stamped.img1_ptr;
    cv_bridge::CvImageConstPtr depth_ptr = cur_rgbd_stamped.img2_ptr;

//    LOG(INFO) << depth_ptr->image.depth() << ", " << depth_ptr->image.channels();

//    double timestamp = rgb_ptr->header.stamp.toSec();
//    std::string image_path = SAVE_PATH + "rgb/" + std::to_string(uint64_t(timestamp * 1e9)) + ".bmp";
//    cv::imwrite(image_path, rgb_ptr->image);
//    timestamp = depth_ptr->header.stamp.toSec();
//    image_path = SAVE_PATH + "depth/" + std::to_string(uint64_t(timestamp * 1e9)) + ".bmp";
//    cv::imwrite(image_path, depth_ptr->image);


    for(int i = 0; i < depth_ptr->image.rows; i++){
        for(int j = 0; j < depth_ptr->image.cols; j++){
            if(depth_ptr->image.at<uint16_t>(i,j) != NAN){
                Eigen::Vector3f point(float(j), float(i), 1.); // Image i,j is optical_frame y,x, not x,y! Invert them when going to point cloud
                point = point * float(depth_ptr->image.at<uint16_t>(i,j)) * 0.001;
                point = K_inv * point;

                pcl::PointXYZRGB ptxyzrgb;
                ptxyzrgb.x = point.x();
                ptxyzrgb.y = point.y();
                ptxyzrgb.z = point.z();
                ptxyzrgb.r = float(rgb_ptr->image.at<cv::Vec3b>(i,j)[0]); // this is assigning bgr to rgb, later in publisher's conversion to ROS msg, rgb here is assigned to bgr
                ptxyzrgb.g = float(rgb_ptr->image.at<cv::Vec3b>(i,j)[1]);
                ptxyzrgb.b = float(rgb_ptr->image.at<cv::Vec3b>(i,j)[2]);
                full_cloud_ptr->push_back(ptxyzrgb);
            }
        }
    }

//    double timestamp = rgb_ptr->header.stamp.toSec();
//    std::string cloud_path = SAVE_PATH + "ball_cloud/" + std::to_string(uint64_t(timestamp * 1e9)) + ".pcd";
//    pcl::io::savePCDFileASCII (cloud_path, *ball_cloud_ptr);
}

void LocalizationFlow::updateParams(){
    MIN_OBJECT_AREA = MIN_OBJ_PIX_DIAM * MIN_OBJ_PIX_DIAM;
    MAX_OBJECT_AREA = FRAME_WIDTH * FRAME_HEIGHT * MAX_OBJ_PERCENTAGE / 100.;
}

void LocalizationFlow::SegmentBall2D(){
    cv_bridge::CvImageConstPtr rgb_ptr = cur_rgbd_stamped.img1_ptr;

    //matrix sotrage for original img with drawn track bars
    cv::Mat cameraFeed = rgb_ptr->image;
    //matrix storage for HSV image
    cv::Mat imgHSV;
    //matrix storage for binary threshold image, imgThresed is a member var

    if(ball_thres_method == "HSV"){
        cvtColor(rgb_ptr->image, imgHSV,COLOR_BGR2HSV);
        //filter HSV image between values and store filtered image to
        //threshold matrix
        inRange(imgHSV,cv::Scalar(H_MIN,S_MIN,V_MIN),cv::Scalar(H_MAX,S_MAX,V_MAX),imgThresed);
    }else if(ball_thres_method == "RGB"){
        inRange(rgb_ptr->image,cv::Scalar(B_MIN,G_MIN,R_MIN),cv::Scalar(B_MAX,G_MAX,R_MAX),imgThresed);
    }else{
        cerr << "Invalid ball_thres_method: " << ball_thres_method  << ". Valid methods: HSV, RGB" << endl;
        return;
    }
    //perform morphological operations on thresholded image to eliminate noise
    //and emphasize the filtered object(s)
    if(useMorphOps)
        morphOps(imgThresed);
    //pass in thresholded frame to our object tracking function
    //this function will return the x and y coordinates of the
    //filtered object
    if(trackObjects)
        trackFilteredObject(ball_j2D,ball_i2D, imgThresed,cameraFeed); // the x,y in this func is j,i in Mat idx

    //show frames
    if(cv_vis){
        imshow(windowName2,imgThresed);
        imshow(windowName,cameraFeed);
        if(ball_thres_method == "HSV")
            imshow(windowName1,imgHSV);

        waitKey(30); //!! mind this 30ms when using openCV window visualization
    }
}

void LocalizationFlow::GetBallCloud(){
    ball_cloud_ptr->clear();
    cv_bridge::CvImageConstPtr depth_ptr = cur_rgbd_stamped.img2_ptr;

    int ROI_radius = min(ball_pix_radius, 20); // when ball is too close and thus too big, only need pt.s around center
    int i_min = max(ball_i2D - ROI_radius, 0);
    int i_max = min(ball_i2D + ROI_radius, FRAME_HEIGHT-1);
    int j_min = max(ball_j2D - ROI_radius, 0);
    int j_max = min(ball_j2D + ROI_radius, FRAME_WIDTH-1);
    Rect ROI(j_min, i_min, j_max - j_min, i_max - i_min); // x,y,width, height
    Mat cropped_depthImg = depth_ptr->image(ROI);

    double min_z, max_z, max_valid_z;
    minMaxLoc(cropped_depthImg, &min_z, &max_z);
    min_z *= 0.001;
    max_z *= 0.001;

    if(max_z - min_z > 2 * ball_real_radius){
        max_valid_z = (min_z + max_z) / 2;
    }else
        max_valid_z = clip_z_dis[1];

    for(int i = i_min; i <= i_max; i++) {
        for (int j = j_min; j <= j_max; j++) {
            if (imgThresed.at<bool>(i, j)) {
                float z = depth_ptr->image.at<uint16_t>(i, j) * 0.001;
                if ((z > clip_z_dis[0]) && (z < max_valid_z)) {
                    Eigen::Vector3f point(float(j), float(i),1.); // Image i,j is optical_frame y,x, not x,y! Invert them when going to point cloud
                    point = point * z;
                    point = K_inv * point; //todo: if needed, this can be speed up by first avg z*[u,v,1] and then multiply by K^-1, but then we won't have ball cloud
                    pcl::PointXYZ pcl_pxyz(point.x(), point.y(), point.z());
                    ball_cloud_ptr->push_back(pcl_pxyz);
                }
            }
        }
    }
}

void LocalizationFlow::CalcBallCenter3D(){ // Get center of ball in world coordinate
    ball_center.setZero();
    size_t size = ball_cloud_ptr->size();
    if(size){
        for(const auto &iter : ball_cloud_ptr->points){
            ball_center(0) += iter.x;
            ball_center(1) += iter.y;
            ball_center(2) += iter.z;
        }
        ball_center(0) = ball_center(0) / size;
        ball_center(1) = ball_center(1) / size;
        ball_center(2) = ball_center(2) / size;

        ball_center = cur_d435i_pos + cur_d435i_ori * ball_center;
        ball_center += ball_center.normalized() * ball_real_radius; // plus the ball radius to get the real ball center
    }else
        cerr << "-------- ball cloud of size 0 when ball is detected! ----------" << endl;
}

std::string LocalizationFlow::intToString(int number){
    std::stringstream ss;
    ss << number;
    return ss.str();
}

void LocalizationFlow::createTrackbars(){
    //create window for trackbars
    namedWindow(trackbarWindowName,0);
    //create memory to store trackbar name on window
    char TrackbarName[50];
    if(ball_thres_method == "HSV"){
        sprintf( TrackbarName, "H_MIN", H_MIN);
        sprintf( TrackbarName, "H_MAX", H_MAX);
        sprintf( TrackbarName, "S_MIN", S_MIN);
        sprintf( TrackbarName, "S_MAX", S_MAX);
        sprintf( TrackbarName, "V_MIN", V_MIN);
        sprintf( TrackbarName, "V_MAX", V_MAX);
    }else if(ball_thres_method == "RGB"){
        sprintf( TrackbarName, "R_MIN", R_MIN);
        sprintf( TrackbarName, "R_MAX", R_MAX);
        sprintf( TrackbarName, "G_MIN", G_MIN);
        sprintf( TrackbarName, "G_MAX", G_MAX);
        sprintf( TrackbarName, "B_MIN", B_MIN);
        sprintf( TrackbarName, "B_MAX", B_MAX);
    }else{
        cerr << "Invalid ball_thres_method: " << ball_thres_method  << ". Valid methods: HSV, RGB" << endl;
        return;
    }
    sprintf( TrackbarName, "ERODE_DIAM", ERODE_DIAM);
    sprintf( TrackbarName, "DILATE_DIAM", DILATE_DIAM);
    sprintf( TrackbarName, "MAX_NUM_OBJECTS", MAX_NUM_OBJECTS);
    sprintf( TrackbarName, "MIN_OBJ_PIX_DIAM", MIN_OBJ_PIX_DIAM);
    sprintf( TrackbarName, "MAX_OBJ_PERCENTAGE", MAX_OBJ_PERCENTAGE);

    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    if(ball_thres_method == "HSV"){
        createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, 256, on_trackbar );
        createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, 256, on_trackbar );
        createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, 256, on_trackbar );
        createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, 256, on_trackbar );
        createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, 256, on_trackbar );
        createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, 256, on_trackbar );
    }else if(ball_thres_method == "RGB"){
        createTrackbar( "R_MIN", trackbarWindowName, &R_MIN, 256, on_trackbar );
        createTrackbar( "R_MAX", trackbarWindowName, &R_MAX, 256, on_trackbar );
        createTrackbar( "G_MIN", trackbarWindowName, &G_MIN, 256, on_trackbar );
        createTrackbar( "G_MAX", trackbarWindowName, &G_MAX, 256, on_trackbar );
        createTrackbar( "B_MIN", trackbarWindowName, &B_MIN, 256, on_trackbar );
        createTrackbar( "B_MAX", trackbarWindowName, &B_MAX, 256, on_trackbar );
    }else{
        cerr << "Invalid ball_thres_method: " << ball_thres_method  << ". Valid methods: HSV, RGB" << endl;
        return;
    }
    createTrackbar( "ERODE_DIAM", trackbarWindowName, &ERODE_DIAM, 30, on_trackbar );
    createTrackbar( "DILATE_DIAM", trackbarWindowName, &DILATE_DIAM, FRAME_HEIGHT, on_trackbar );
    createTrackbar( "MAX_NUM_OBJECTS", trackbarWindowName, &MAX_NUM_OBJECTS, 50, on_trackbar );
    createTrackbar( "MIN_OBJ_PIX_DIAM", trackbarWindowName, &MIN_OBJ_PIX_DIAM, 30, on_trackbar );
    createTrackbar( "MAX_OBJ_OCCUPATION", trackbarWindowName, &MAX_OBJ_PERCENTAGE, 100, on_trackbar );
}

void LocalizationFlow::drawObject(int x, int y, Mat &frame){

    //use some of the openCV drawing functions to draw crosshairs
    //on your tracked image!

    //UPDATE:JUNE 18TH, 2013
    //added 'if' and 'else' statements to prevent
    //memory errors from writing off the screen (ie. (-25,-25) is not within the window!)

    circle(frame,Point(x,y),20,Scalar(0,255,0),2);
    if(y-25>0)
        line(frame,Point(x,y),Point(x,y-25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,0),Scalar(0,255,0),2);
    if(y+25<FRAME_HEIGHT)
        line(frame,Point(x,y),Point(x,y+25),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(x,FRAME_HEIGHT),Scalar(0,255,0),2);
    if(x-25>0)
        line(frame,Point(x,y),Point(x-25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(0,y),Scalar(0,255,0),2);
    if(x+25<FRAME_WIDTH)
        line(frame,Point(x,y),Point(x+25,y),Scalar(0,255,0),2);
    else line(frame,Point(x,y),Point(FRAME_WIDTH,y),Scalar(0,255,0),2);

    putText(frame,intToString(x)+","+intToString(y),Point(x,y+30),1,1,Scalar(0,255,0),2);

}

void LocalizationFlow::morphOps(cv::Mat &thresh){

    //create structuring element that will be used to "dilate" and "erode" image.
    //the element chosen here is a 3px by 3px rectangle

    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(ERODE_DIAM,ERODE_DIAM));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(DILATE_DIAM,DILATE_DIAM));

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);

    dilate(thresh,thresh,dilateElement);
//    dilate(thresh,thresh,dilateElement);
}

void LocalizationFlow::trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed){

    Mat temp;
    threshold.copyTo(temp);
    //these two vectors needed for output of findContours
    std::vector< std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    //find contours of filtered image using openCV findContours function
    findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );
    //use moments method to find our filtered object
    double maxArea = 0;
    ball_pix_radius = 0;
    found_ball = false;

    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
//        LOG(INFO) << "numObjects: " << numObjects;
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>maxArea){
                    x = moment.m10/area;
                    y = moment.m01/area;
                    found_ball = true;
                    maxArea = area;
                }
            }
            //let user know you found an object
            if(found_ball){
                ball_pix_radius = sqrt(maxArea / M_PI);
                if(cv_vis){
                    putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
                    //draw object location on screen
                    drawObject(x,y,cameraFeed);
                }
            }
        }else
            putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
}