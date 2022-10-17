#include "localization_flow.h"
#include "localization_config.h"
#include <pcl/common/transforms.h>
#include "glog/logging.h"

LocalizationFlow::LocalizationFlow(ros::NodeHandle &nh){
    // subscriber
    rgb_sub_ptr_ = std::make_shared<ImgSubscriber>(nh, "/d435i/color/image_raw", 100);
    depth_sub_ptr_ = std::make_shared<ImgSubscriber>(nh, "/d435i/aligned_depth_to_color/image_raw", 100);
    // publisher
    T_rgbd_ball_broadcast_ptr_ = std::make_shared<TFBroadCaster>("/d435i_color_optical_frame", "/ball");

    // Get Cam Cali Matrix K
    K << 921.568359375, 0.0, 643.44873046875,
         0.0, 919.7422485351562, 379.1036071777344,
         0.0, 0.0, 1.0;
}



void LocalizationFlow::Run() {
    // ReadData
    img_sub_ptr_->ParseData(imgBuffer);
    while(!imgBuffer.empty()){ // HasData
        curr_cloud.cloud_ptr->clear();
        curr_cloud = cloud_data_buffer_.back(); // process the last cloud only, to ensure real-time
        cloud_data_buffer_.clear();
        // Calibrate Cloud:
        caliCurrPointCloud();
        // Publish calibrated point cloud
        calied_cloud_pub_ptr_->Publish(curr_cloud.cloud_ptr, curr_cloud.time);
        // filter out points that are outside the pool xy range or lower than the water surface
        filterBoat();
        boat_cloud_pub_ptr_->Publish(boat_cloud.cloud_ptr, boat_cloud.time);
        // filter out red and green LED clouds
        filterLED();
        // publish LED clouds
        red_cloud_pub_ptr_->Publish(red_cloud.cloud_ptr, red_cloud.time);
        green_cloud_pub_ptr_->Publish(green_cloud.cloud_ptr, green_cloud.time);
        // Locate LED and boat
        if(localize())
            publishLocalizationData();

    }
    return false;
}



Eigen::Vector3f LocalizationFlow::getCenter(const CloudData &cloud) {
    Eigen::Vector3f center(0.,0.,0.);
    for(const auto &iter : cloud.cloud_ptr->points){
        center(0) += iter.x / cloud.cloud_ptr->size();
        center(1) += iter.y / cloud.cloud_ptr->size();
        center(2) += iter.z / cloud.cloud_ptr->size();
    }
    return center;
}

void LocalizationFlow::publishLocalizationData() {
    // publish localization info for control
    localization_publisher_ptr->Publish(boat_state, boat_vel, curr_cloud.time);

    // visualize boat vel
    Eigen::Vector3f position = boat_state;
    position(2) = 0;
//        LOG(INFO) << "vy = " << boat_vel.y() << ", vx = " << boat_vel.x() << ", orientation = " << atan2(boat_vel.y(), boat_vel.x());
    if(boat_vel.block<2,1>(0,0).norm() > 0.00001) // update vis only if vel != 0
        boat_vel_vis_pub_ptr->Publish(position, atan2(boat_vel.y(), boat_vel.x()), curr_cloud.time);

    // publish LED TF
    Eigen::Matrix4f T_dock_red = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f T_dock_green = Eigen::Matrix4f::Identity();
    T_dock_red.block<3,1>(0,3) = R_center;
    T_dock_green.block<3,1>(0,3) = G_center;
    red_tf_pub_ptr_->SendTransform(T_dock_red, red_cloud.time);
    green_tf_pub_ptr_->SendTransform(T_dock_green, green_cloud.time);

    // publish boat TF
    Eigen::Matrix4f T_dock_boat = Eigen::Matrix4f::Identity();
    T_dock_boat << cos(boat_state(2)), -sin(boat_state(2)), 0, boat_state.x(),
            sin(boat_state(2)),  cos(boat_state(2)), 0, boat_state.y(),
            0,                            0, 1,              0,
            0,                            0, 0,              1;
    boat_tf_pub_ptr_->SendTransform(T_dock_boat, curr_cloud.time);
}