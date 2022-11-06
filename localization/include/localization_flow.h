#ifndef SRC_LOCALIZATION_FLOW_H
#define SRC_LOCALIZATION_FLOW_H

#include <ros/ros.h>
// subscriber
#include "subscriber/dual_img_subscriber.hpp"
#include "subscriber/tf_listener.hpp"
// publisher
#include "publisher/cloud_publisher.hpp"
#include "publisher/pose_publisher.hpp"
#include "publisher/tf_broadcaster.hpp"
// sensor data
#include <sensor_msgs/CameraInfo.h>
#include "sensor_data/cloud_data.hpp"
// yaml
#include <yaml-cpp/yaml.h>
// tools
#include "tools/tic_toc.hpp"

class LocalizationFlow{
public:
    LocalizationFlow(ros::NodeHandle &nh);

    void Run();

    // for use
    void SegmentBallThreshold();
    void SegmentBallKMeans();
    // for rviz
    void GenerateFullPointCloud();

    void GetBallCenter();

private:
    // node handle
    ros::NodeHandle nh_;
    // calibration
    Eigen::Matrix3f K_inv;
    // subscriber
    std::shared_ptr<DualImgSubscriber> rgb_d_sub_ptr_;
    std::shared_ptr<TFListener> tf_listener_ptr_;
    // publisher
    //   for use
    std::shared_ptr<TFBroadCaster> tf_broadcast_ptr_;
    //   for rviz
    std::shared_ptr<CloudPublisher> full_cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> ball_cloud_pub_ptr_;

    // params
    float clip_z_dis[2]; // min, max in meters, according to datasheet
    uint8_t threshold;

    // data processing flow
    bool found_ball;
    Eigen::Vector3f prev_d435i_pos_wth_ball;
    Eigen::Quaternionf prev_d435i_ori_wth_ball;
    Eigen::Vector3f prev_ball_center;
    Eigen::Vector3f cur_d435i_pos;
    Eigen::Quaternionf cur_d435i_ori;
    ros::Time cur_d435i_time;
    std::deque<std::pair<cv_bridge::CvImageConstPtr, cv_bridge::CvImageConstPtr>> rgb_d_buffer_;

    // for use
    pcl::PointCloud<pcl::PointXYZ>::Ptr ball_cloud_ptr;
    Eigen::Vector3f ball_center;
    // for rviz
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud_ptr;


    // timing
//    std::shared_ptr<TicToc> time_cali;


};

#endif //SRC_LOCALIZATION_FLOW_H