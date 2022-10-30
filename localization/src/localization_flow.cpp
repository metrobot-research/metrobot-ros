#include "localization_flow.h"
#include "localization_config.h"
#include <pcl/common/transforms.h>
#include "glog/logging.h"


LocalizationFlow::LocalizationFlow(ros::NodeHandle &nh):
    nh_(nh),
    rgb_d_sub_ptr_(std::make_shared<DualImgSubscriber>(nh_, "/color/image_raw", "/aligned_depth_to_color/image_raw", 20)),
    tf_listener_ptr_(std::make_shared<TFListener>()),
    tf_broadcast_ptr_(std::make_shared<TFBroadCaster>()),
    cloud_pub_ptr_(std::make_shared<CloudPublisher>(nh_, "/ball_cloud", "/D435i", 20)),
    ball_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()){
    K << 921.568359375, 0.0, 643.44873046875,
         0.0, 919.7422485351562, 379.1036071777344,
         0.0, 0.0, 1.0;
}



void LocalizationFlow::Run() {
    // ReadData
    tf_listener_ptr_->lookupTransform("/t265_odom_frame", "d435i_color_optical_frame", cur_d435i_pos, cur_d435i_ori);
    rgb_d_sub_ptr_->ParseData(rgb_d_buffer_);

    while(!rgb_d_buffer_.empty()){ // HasData
        SegmentBallThreshold(); // TODO: put RGB xy and Depth as xyz in ball_cloud_ptr
//        SegmentBallKMeans();
        BallCloud3DReconstruction(); // TODO: use K to transform the ball_cloud_ptr to real 3D coordinate, publish the ball cloud for checking only
        PublishBallLocation(); // TODO: calc center of ball cloud wrt d435i, transform to world coordinate, and publish ball tf with identity rotation
    }
}

void LocalizationFlow::SegmentBallThreshold() {}

void LocalizationFlow::SegmentBallKMeans() {}

void LocalizationFlow::BallCloud3DReconstruction() {}

void LocalizationFlow::PublishBallLocation() {}