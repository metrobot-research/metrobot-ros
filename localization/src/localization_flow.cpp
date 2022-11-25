#include "localization_flow.h"
//#include "localization_config.h"
#include <pcl/common/transforms.h>
#include "glog/logging.h"
#include "global_definition/global_definition.h"

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>
#include "tools/tic_toc.hpp"

//#define CHECK_POINTCLOUD

LocalizationFlow::LocalizationFlow(ros::NodeHandle &nh):
    nh_(nh),
    rgb_d_sub_ptr_(std::make_shared<DualImgSubscriber>(nh_, "/d435i/color/image_raw", "/d435i/aligned_depth_to_color/image_raw", 20)),
    tf_listener_ptr_(std::make_shared<TFListener>()),
    tf_broadcast_ptr_(std::make_shared<TFBroadCaster>()),
    full_cloud_pub_ptr_(std::make_shared<CloudPublisher>(nh_, "/full_cloud", "/d435i_color_optical_frame", 20)),
    ball_cloud_pub_ptr_(std::make_shared<CloudPublisher>(nh_, "/ball_cloud", "/d435i_color_optical_frame", 20)),
    found_ball(false),
    full_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>()),
    ball_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()),
    time_run(std::make_shared<TicToc>("time per Run(): ", true)){

    Eigen::Matrix3f K;
    /* align depth to color on
     * use align depth to color's K, which is the same as the K of color
     * different resolution, align_depth on/off  -->  different K */
    //// 1280 * 720
    K << 921.568359375, 0.0, 643.44873046875,
         0.0, 919.7422485351562, 379.1036071777344,
         0.0, 0.0, 1.0;
    clip_z_dis[0] = 0.28;
    //// 848 * 480
//    K << 614.37890625, 0.0, 426.29913330078125,
//         0.0, 613.1614990234375, 252.73573303222656,
//         0.0, 0.0, 1.0;
//    clip_z_dis[0] = 0.195;
    //// 640 * 360
//    K << 460.7841796875, 0.0, 321.724365234375,
//         0.0, 459.8711242675781, 189.5518035888672,
//         0.0, 0.0, 1.0;
//    clip_z_dis[0] = 0.175;

    nh_.getParam("camera2_clip_distance", clip_z_dis[1]);
    if(clip_z_dis[1] < 0)
        clip_z_dis[1] = 30;
    nh_.getParam("H_MIN", H_MIN);
    nh_.getParam("H_MAX", H_MAX);
    nh_.getParam("S_MIN", S_MIN);
    nh_.getParam("S_MAX", S_MAX);
    nh_.getParam("V_MIN", V_MIN);
    nh_.getParam("V_MAX", V_MAX);
    nh_.getParam("camera2_color_width", FRAME_WIDTH);
    nh_.getParam("camera2_color_height", FRAME_HEIGHT);

    K_inv = K.inverse();
}



void LocalizationFlow::Run(){
    time_run->tic();
    rgb_d_sub_ptr_->ParseData(rgb_d_buffer_);

    if(!rgb_d_buffer_.empty()){
        cur_rgbd_stamped = rgb_d_buffer_.back();
        rgb_d_buffer_.clear();

        try{
            tf_listener_ptr_->lookupTransform("/t265_odom_frame", "/d435i_color_optical_frame", cur_rgbd_stamped.time, cur_d435i_pos, cur_d435i_ori);
        }catch(tf2::LookupException &exc){ // wait until the camera boot node has fully started
            return;
        }catch(tf2::ConnectivityException &exc){
            return;
        }catch(tf2::ExtrapolationException &exc){
            return;
        }

#ifdef CHECK_POINTCLOUD
        // Publish the whole point cloud for checking with Realsense point cloud.
        // Remember to enable "pointcloud" filter to publish Realsense point cloud
        full_cloud_ptr->clear();
        GenerateFullPointCloud();
        full_cloud_pub_ptr_->Publish(full_cloud_ptr, cur_rgbd_stamped.time);
#else
        SegmentBallThreshold();
//        SegmentBallKMeans();

        if(!ball_cloud_ptr->empty()){ // result stored in "ball_center", wrt world coordinate, expressed in world coordinate
                             // modifies ball_cloud
            found_ball = true;
            GetBallCenter();
            ball_cloud_pub_ptr_->Publish(ball_cloud_ptr, cur_rgbd_stamped.time);
            // Publish ball tf wrt world frame, with orientation set to identity for convenience
            tf_broadcast_ptr_->SendTransform("/t265_odom_frame", "/ball_real",
                                             ball_center, Eigen::Quaternionf::Identity(), cur_rgbd_stamped.time);
        }
#endif
    }
    time_run->toc();
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
                ptxyzrgb.r = float(rgb_ptr->image.at<cv::Vec3b>(i,j)[0]); // fixme: guess this is assigning bgr to rgb, later in publisher's conversion to ROS msg, rgb here is assigned to bgr
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

void LocalizationFlow::SegmentBallThreshold(){
    ball_cloud_ptr->clear();

    cv_bridge::CvImageConstPtr rgb_ptr = cur_rgbd_stamped.img1_ptr;
    cv_bridge::CvImageConstPtr depth_ptr = cur_rgbd_stamped.img2_ptr;

    //matrix sotrage for original img with drawn track bars
    cv::Mat cameraFeed = rgb_ptr->image;
    //matrix storage for HSV image
    cv::Mat imgHSV;
    //matrix storage for binary threshold image
    cv::Mat imgThresed;
    //x and y values for the location of the object
    int x=0, y=0;
    //create slider bars for HSV filtering
    createTrackbars();

    cvtColor(rgb_ptr->image, imgHSV,COLOR_BGR2HSV);
    //filter HSV image between values and store filtered image to
    //threshold matrix
    inRange(imgHSV,cv::Scalar(H_MIN,S_MIN,V_MIN),cv::Scalar(H_MAX,S_MAX,V_MAX),imgThresed);
    //perform morphological operations on thresholded image to eliminate noise
    //and emphasize the filtered object(s)
    if(useMorphOps)
        morphOps(imgThresed);
    //pass in thresholded frame to our object tracking function
    //this function will return the x and y coordinates of the
    //filtered object
    if(trackObjects)
        trackFilteredObject(x,y, imgThresed,cameraFeed);

    //show frames
    imshow(windowName2,imgThresed);
    imshow(windowName,cameraFeed);
    imshow(windowName1,imgHSV);
    waitKey(30); //fixme: only used for window visualization, remove this for final application

    //TODO: create ball cloud using imgThresed, depth_ptr, K_inv

//    for(int i = 0; i < depth_ptr->image.rows; i++){
//        for(int j = 0; j < depth_ptr->image.cols; j++){
//            if(depth_ptr->image.at<uint16_t>(i,j) != NAN){
//                uint8_t bgr[3] = {rgb_ptr->image.at<cv::Vec3b>(i,j)[0],
//                                   rgb_ptr->image.at<cv::Vec3b>(i,j)[1],
//                                   rgb_ptr->image.at<cv::Vec3b>(i,j)[2]}; // fixme: guess it's bgr here, output full cloud is correct because during publishing there's a conversion, where rgb are assigned to bgr
//                if(sqrt((bgr[0]*bgr[0] + bgr[1]*bgr[1] + bgr[2]*bgr[2]) / 3.) > bright_thres){
//                    Eigen::Vector3f point(float(j), float(i), 1.); // Image i,j is optical_frame y,x, not x,y! Invert them when going to point cloud
//                    point = point * float(depth_ptr->image.at<uint16_t>(i,j)) * 0.001;
//                    if(point.z() < clip_z_dis[0] || point.z() > clip_z_dis[1]) // clip out invalid points, K_inv does not change z coord
//                        continue;
//                    point = K_inv * point;
//
//                    if(found_ball){
//                        // remove points too far from predicted ball pos wrt d435i according to tracking camera
//                        ball_center_pred = cur_d435i_ori.inverse() * (prev_ball_center - cur_d435i_pos); // in d435i frame, fixme: assuming ball is static wrt world frame
//                                                                                                         // fixme: difficult to distract, but once distracted, lost forever
//                        if((point - ball_center_pred).norm() < pred_thres){
//                            pcl::PointXYZ pcl_pxyz(point.x(), point.y(), point.z());
//                            ball_cloud_ptr->push_back(pcl_pxyz);
//                        }
//                    }else{
//                        pcl::PointXYZ pcl_pxyz(point.x(), point.y(), point.z());
//                        ball_cloud_ptr->push_back(pcl_pxyz);
//                    }
//                }
//            }
//        }
//    }
}

void LocalizationFlow::GetBallCenter(){ // Get center of ball in d435i coordinate
    ball_center.setZero();
    size_t size = ball_cloud_ptr->size();
    for(const auto &iter : ball_cloud_ptr->points){
        ball_center(0) += iter.x;
        ball_center(1) += iter.y;
        ball_center(2) += iter.z;
    }
    ball_center(0) = ball_center(0) / size;
    ball_center(1) = ball_center(1) / size;
    ball_center(2) = ball_center(2) / size;

    ball_center = cur_d435i_pos + cur_d435i_ori * ball_center;
    prev_ball_center = ball_center;
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
    sprintf( TrackbarName, "H_MIN", H_MIN);
    sprintf( TrackbarName, "H_MAX", H_MAX);
    sprintf( TrackbarName, "S_MIN", S_MIN);
    sprintf( TrackbarName, "S_MAX", S_MAX);
    sprintf( TrackbarName, "V_MIN", V_MIN);
    sprintf( TrackbarName, "V_MAX", V_MAX);
    //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
    //the max value the trackbar can move (eg. H_HIGH),
    //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );


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

    Mat erodeElement = getStructuringElement( MORPH_RECT,Size(3,3));
    //dilate with larger element so make sure object is nicely visible
    Mat dilateElement = getStructuringElement( MORPH_RECT,Size(8,8));

    erode(thresh,thresh,erodeElement);
    erode(thresh,thresh,erodeElement);


    dilate(thresh,thresh,dilateElement);
    dilate(thresh,thresh,dilateElement);



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
    double refArea = 0;
    bool objectFound = false;
    if (hierarchy.size() > 0) {
        int numObjects = hierarchy.size();
        //if number of objects greater than MAX_NUM_OBJECTS we have a noisy filter
        if(numObjects<MAX_NUM_OBJECTS){
            for (int index = 0; index >= 0; index = hierarchy[index][0]) {

                Moments moment = moments((cv::Mat)contours[index]);
                double area = moment.m00;

                //if the area is less than 20 px by 20px then it is probably just noise
                //if the area is the same as the 3/2 of the image size, probably just a bad filter
                //we only want the object with the largest area so we safe a reference area each
                //iteration and compare it to the area in the next iteration.
                if(area>MIN_OBJECT_AREA && area<MAX_OBJECT_AREA && area>refArea){
                    x = moment.m10/area;
                    y = moment.m01/area;
                    objectFound = true;
                    refArea = area;
                }else objectFound = false;


            }
            //let user know you found an object
            if(objectFound){
                putText(cameraFeed,"Tracking Object",Point(0,50),2,1,Scalar(0,255,0),2);
                //draw object location on screen
                drawObject(x,y,cameraFeed);}

        }else putText(cameraFeed,"TOO MUCH NOISE! ADJUST FILTER",Point(0,50),1,2,Scalar(0,0,255),2);
    }
}