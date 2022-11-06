#include "localization_flow.h"
//#include "localization_config.h"
#include <pcl/common/transforms.h>
#include "glog/logging.h"
#include "global_definition/global_definition.h"

#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/pcd_io.h>

//#define CHECK_POINTCLOUD

LocalizationFlow::LocalizationFlow(ros::NodeHandle &nh):
    nh_(nh),
    rgb_d_sub_ptr_(std::make_shared<DualImgSubscriber>(nh_, "/d435i/color/image_raw", "/d435i/aligned_depth_to_color/image_raw", 20)),
    tf_listener_ptr_(std::make_shared<TFListener>()),
    tf_broadcast_ptr_(std::make_shared<TFBroadCaster>()),
    full_cloud_pub_ptr_(std::make_shared<CloudPublisher>(nh_, "/full_cloud", "/d435i_color_optical_frame", 20)),
    ball_cloud_pub_ptr_(std::make_shared<CloudPublisher>(nh_, "/ball_cloud", "/d435i_color_optical_frame", 20)),
    bright_threshold(200),
    found_ball(false),
    full_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>()),
    ball_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()){
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

    nh_.param<float>("camera2_clip_distance", clip_z_dis[1], -2);
    if(clip_z_dis[1] < 0)
        clip_z_dis[1] = 30;

    K_inv = K.inverse();
}



void LocalizationFlow::Run(){
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
}

void LocalizationFlow::SegmentBallThreshold(){
    ball_cloud_ptr->clear();

    cv_bridge::CvImageConstPtr rgb_ptr = cur_rgbd_stamped.img1_ptr;
    cv_bridge::CvImageConstPtr depth_ptr = cur_rgbd_stamped.img2_ptr;

    for(int i = 0; i < depth_ptr->image.rows; i++){
        for(int j = 0; j < depth_ptr->image.cols; j++){
            if(depth_ptr->image.at<uint16_t>(i,j) != NAN){
                uint8_t bgr[3] = {rgb_ptr->image.at<cv::Vec3b>(i,j)[0],
                                   rgb_ptr->image.at<cv::Vec3b>(i,j)[1],
                                   rgb_ptr->image.at<cv::Vec3b>(i,j)[2]}; // fixme: guess it's bgr here, output full cloud is correct because during publishing there's a conversion, where rgb are assigned to bgr
                if(bgr[0] > bright_threshold && bgr[1] > bright_threshold && bgr[2] > bright_threshold){
                    Eigen::Vector3f point(float(j), float(i), 1.); // Image i,j is optical_frame y,x, not x,y! Invert them when going to point cloud
                    point = point * float(depth_ptr->image.at<uint16_t>(i,j)) * 0.001;
                    if(point.z() < clip_z_dis[0] || point.z() > clip_z_dis[1]) // clip out invalid points, K_inv does not change z coord
                        continue;
                    point = K_inv * point;

                    if(found_ball){
                        // remove points too far from predicted ball pos wrt d435i according to tracking camera
                        ball_center_pred = cur_d435i_ori.inverse() * (prev_ball_center - cur_d435i_pos); // in d435i frame, fixme: assuming ball is static wrt world frame
                                                                                                         // fixme: difficult to distract, but once distracted, lost forever
                        if((point - ball_center_pred).norm() < 0.2){
                            pcl::PointXYZ pcl_pxyz(point.x(), point.y(), point.z());
                            ball_cloud_ptr->push_back(pcl_pxyz);
                        }
                    }else{
                        pcl::PointXYZ pcl_pxyz(point.x(), point.y(), point.z());
                        ball_cloud_ptr->push_back(pcl_pxyz);
                    }
                }
            }
        }
    }
}

void LocalizationFlow::SegmentBallKMeans(){}

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

bool LocalizationFlow::GetBallCenter(){ // Get center of ball in d435i coordinate
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