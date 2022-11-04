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

    void SegmentBallThreshold();
    void SegmentBallKMeans();

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
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;

    // data processing flow
    Eigen::Vector3f cur_d435i_pos;
    Eigen::Quaternionf cur_d435i_ori;
    ros::Time cur_d435i_time;
    std::deque<std::pair<cv_bridge::CvImageConstPtr, cv_bridge::CvImageConstPtr>> rgb_d_buffer_;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ball_cloud_ptr;
    Eigen::Vector3f ball_center;

    // timing
//    std::shared_ptr<TicToc> time_cali;


};

#endif //SRC_LOCALIZATION_FLOW_H