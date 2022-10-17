#include "subscriber/dual_img_subscriber.hpp"
#include <boost/bind.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

DualImgSubscriber::DualImgSubscriber(ros::NodeHandle &nh, std::string topic1, std::string topic2, size_t buff_size)
        : nh_(nh){
    sub_1.subscribe(nh_, topic1, 10);
    sub_2.subscribe(nh_, topic2, 10);
    sync_.connectInput(sub_1, sub_2);
    sync_.registerCallback(boost::bind(&DualImgSubscriber::msg_callback), this, _1, _2);
}