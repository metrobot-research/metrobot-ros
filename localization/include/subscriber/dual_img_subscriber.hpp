#ifndef SUBSCRIBER_DUAL_IMG_SUBSCRIBER_H
#define SUBSCRIBER_DUAL_IMG_SUBSCRIBER_H

#include <deque>
#include <mutex>
#include <thread>
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>



class DualImgSubscriber{
public:
    DualImgSubscriber(ros::NodeHandle &nh, std::string topic1, std::string topic2, size_t buff_size);

    DualImgSubscriber() = default;

    void ParseData(std::deque<std::pair<cv_bridge::CvImageConstPtr, cv_bridge::CvImageConstPtr>> &target_img_buffer);

private:
    void msg_callback(const sensor_msgs::ImageConstPtr &rgbMsg, const sensor_msgs::ImageConstPtr &depthMsg);

    ros::NodeHandle nh_;
    message_filters::Subscriber<sensor_msgs::ImageConstPtr> sub_1;
    message_filters::Subscriber<sensor_msgs::ImageConstPtr> sub_2;
    message_filters::TimeSynchronizer<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> sync_;
    std::deque<std::pair<cv_bridge::CvImageConstPtr, cv_bridge::CvImageConstPtr>> imgBuffer;

    std::mutex buff_mutex_;
};

#endif // SUBSCRIBER_DUAL_IMG_SUBSCRIBER_H
