#include "subscriber/dual_img_subscriber.hpp"
#include <boost/bind.hpp>

DualImgSubscriber::DualImgSubscriber(ros::NodeHandle &nh, std::string topic1, std::string topic2, size_t buff_size)
        : nh_(nh),
          sub_1(nh_, topic1, buff_size),
          sub_2(nh_, topic2, buff_size),
          sync_(sub_1, sub_2, buff_size*2){
    sync_.registerCallback(boost::bind(&DualImgSubscriber::msg_callback, this, _1, _2));
}

void DualImgSubscriber::ParseData(
        std::deque<std::pair<cv_bridge::CvImageConstPtr, cv_bridge::CvImageConstPtr>> &target_imgBuffer) {
    buff_mutex_.lock();
    if(!imgBuffer.empty()){
        target_imgBuffer.insert(target_imgBuffer.end(), imgBuffer.begin(), imgBuffer.end());
        imgBuffer.clear();
    }
    buff_mutex_.unlock();
}

void DualImgSubscriber::msg_callback(const sensor_msgs::ImageConstPtr &msg1,
                                     const sensor_msgs::ImageConstPtr &msg2) {
    buff_mutex_.lock();
// Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ImgPtr1, cv_ImgPtr2;
    try {
        cv_ImgPtr1 = cv_bridge::toCvShare(msg1);
        cv_ImgPtr2 = cv_bridge::toCvShare(msg2);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    // only when saving imgs, use the following color converter
//    if (cv_ImgPtr1->image.type() == CV_8UC3) // only for RGB image, convert BGR to RGB, grey img is of type CV_8UC1
//        cv::cvtColor(cv_ImgPtr1->image, cv_ImgPtr1->image, cv::COLOR_BGR2RGB);
    imgBuffer.emplace_back(cv_ImgPtr1, cv_ImgPtr2);
    buff_mutex_.unlock();
}