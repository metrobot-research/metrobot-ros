#include "localization_flow.h"
//#include "localization_config.h"
#include <pcl/common/transforms.h>
#include "glog/logging.h"
#include "global_definition/global_definition.h"

#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>


LocalizationFlow::LocalizationFlow(ros::NodeHandle &nh):
    nh_(nh),
    rgb_d_sub_ptr_(std::make_shared<DualImgSubscriber>(nh_, "/d435i/color/image_raw", "/d435i/aligned_depth_to_color/image_raw", 20)),
    tf_listener_ptr_(std::make_shared<TFListener>()),
    tf_broadcast_ptr_(std::make_shared<TFBroadCaster>()),
    cloud_pub_ptr_(std::make_shared<CloudPublisher>(nh_, "/ball_cloud", "/d435i_color_optical_frame", 20)),
    ball_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>()){
    Eigen::Matrix3f K;
//    K << 921.568359375, 0.0, 643.44873046875,
//         0.0, 919.7422485351562, 379.1036071777344,
//         0.0, 0.0, 1.0;
    K << 642.2916870117188, 0.0, 637.2142944335938,
         0.0, 642.2916870117188, 362.7745361328125,
         0.0, 0.0, 1.0;
    K_inv = K.inverse();
}



void LocalizationFlow::Run(){
    // ReadData
//    tf_listener_ptr_->lookupTransform("/t265_odom_frame", "/d435i_color_optical_frame", cur_d435i_pos, cur_d435i_ori, cur_d435i_time);
    rgb_d_sub_ptr_->ParseData(rgb_d_buffer_);

    while(!rgb_d_buffer_.empty()){ // HasData
        SegmentBallThreshold();
        rgb_d_buffer_.clear();
//        SegmentBallKMeans();

        // Publish segmented ball cloud, for visualize only
        cloud_pub_ptr_->Publish(ball_cloud_ptr);

//        GetBallCenter(); // TODO: calc center of ball cloud wrt d435i, transform to world coordinate, and publish ball tf with identity rotation
//        // Publish ball tf with orientation set to identity for convenience
//        tf_broadcast_ptr_->SendTransform("/t265_odom_frame", "/ball_real", center, Eigen::Quaternionf::Identity(), cur_d435i_time);
    }
}

void LocalizationFlow::SegmentBallThreshold(){
    ball_cloud_ptr->clear();

    cv_bridge::CvImageConstPtr rgb_ptr = rgb_d_buffer_.back().first;
    cv_bridge::CvImageConstPtr depth_ptr = rgb_d_buffer_.back().second;

//    LOG(INFO) << depth_ptr->image.depth() << ", " << depth_ptr->image.channels();

//    double timestamp = rgb_ptr->header.stamp.toSec();
//    std::string image_path = SAVE_PATH + "rgb/" + std::to_string(uint64_t(timestamp * 1e9)) + ".bmp";
//    cv::imwrite(image_path, rgb_ptr->image);
//    timestamp = depth_ptr->header.stamp.toSec();
//    image_path = SAVE_PATH + "depth/" + std::to_string(uint64_t(timestamp * 1e9)) + ".bmp";
//    cv::imwrite(image_path, depth_ptr->image);

    // TODO: threshold the rgb images to get the ball, traversing for loop is for checking
    for(int i = 0; i < depth_ptr->image.rows; i++){
        for(int j = 0; j < depth_ptr->image.cols; j++){
            if(depth_ptr->image.at<uint16_t>(i,j) != NAN){
                Eigen::Vector3f point(float(i), float(j), 1.);
                point = point * float(depth_ptr->image.at<uint16_t>(i,j)) * 0.001;
                point = K_inv * point;
                Eigen::Matrix3f R;
                R << 0, 1, 0,
                     1,  0, 0,
                     0,  0, 1;
                point = R * point;

                pcl::PointXYZRGB ptxyzrgb;
                ptxyzrgb.x = point.x();
                ptxyzrgb.y = point.y();
                ptxyzrgb.z = point.z();
                ptxyzrgb.r = float(rgb_ptr->image.at<cv::Vec3b>(i,j)[0]);
                ptxyzrgb.g = float(rgb_ptr->image.at<cv::Vec3b>(i,j)[1]);
                ptxyzrgb.b = float(rgb_ptr->image.at<cv::Vec3b>(i,j)[2]);
                ball_cloud_ptr->push_back(ptxyzrgb);
            }
        }
    }

//    double timestamp = rgb_ptr->header.stamp.toSec();
//    std::string cloud_path = SAVE_PATH + "ball_cloud/" + std::to_string(uint64_t(timestamp * 1e9)) + ".pcd";
//    pcl::io::savePCDFileASCII (cloud_path, *ball_cloud_ptr);
}

void LocalizationFlow::SegmentBallKMeans(){}

void LocalizationFlow::GetBallCenter(){
    // Get center of ball in d435i coordinate
    ball_center.setZero();
    for(const auto &iter : ball_cloud_ptr->points){
        ball_center(0) += iter.x / ball_cloud_ptr->size();
        ball_center(1) += iter.y / ball_cloud_ptr->size();
        ball_center(2) += iter.z / ball_cloud_ptr->size();
    }

    // Transform ball position to world coordinate
    ball_center = cur_d435i_pos + cur_d435i_ori * ball_center;
}