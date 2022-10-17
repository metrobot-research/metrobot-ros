//
// Created by warren on 2021/10/26.
//

#ifndef SRC_LOCALIZATION_FLOW_H
#define SRC_LOCALIZATION_FLOW_H

#include <ros/ros.h>
// subscriber
#include "subscriber/img_subscriber.hpp"
// publisher
#include "publisher/cloud_publisher.hpp"
#include "publisher/localization_publisher.hpp"
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
    Eigen::Vector3f getCenter(const CloudData &cloud);
    // EFFECTS: returns the xyz center


private:
    // calibration
    Eigen::Matrix3f K;
    // subscriber
    std::shared_ptr<ImgSubscriber> rgb_sub_ptr_;
    std::shared_ptr<ImgSubscriber> depth_sub_ptr_;
    // publisher
    std::shared_ptr<TFBroadCaster> T_rgbd_ball_broadcast_ptr_;

    // data processing flow
    std::deque<cv::Mat> rgbBuffer;
    std::deque<cv::Mat> depthBuffer;
    cv::Mat current_rgb;
    cv::Mat current_depth;

    // ball pos wrt rgbd expressed in rgbd
    Eigen::Vector3f ball_pos;

    // timing
//    std::shared_ptr<TicToc> time_cali;


};

#endif //SRC_LOCALIZATION_FLOW_H