#include "localization_config.h"

// constants
// box filter
float Config::x_min;
float Config::x_max;
float Config::y_min;
float Config::y_max;
float Config::z_min;
// brightness filter
float Config::brightnessStrictness;
// LED Separation
float Config::RStrcitnessMax;
float Config::RStrictnessMin;
float Config::GStrcitnessMax;
float Config::GStrictnessMin;
// noise removal
float Config::min_valid_vel;
float Config::max_valid_y_vel;
float Config::min_LED_dis;
float Config::max_LED_dis;
// finish criteria
Eigen::Vector3f Config::finishPos;
float Config::finishThreshold;

// topics
YAML::Node Config::config_node_ = YAML::Node();
std::string Config::rgb_topic;
std::string Config::depth_topic;
std::string Config::raw_cloud_topic;
std::string Config::calied_cloud_topic;
std::string Config::boat_cloud_topic;
std::string Config::red_cloud_topic;
std::string Config::green_cloud_topic;
std::string Config::localization_topic;
std::string Config::boat_vel_vis_topic;
std::string Config::to_control_topic;
std::string Config::rgb_subpath;
std::string Config::depth_subpath;
std::string Config::rosbag_subpath;

Eigen::Matrix3f Config::R_MATLAB;
Eigen::Vector3f Config::p_MATLAB;
Eigen::Matrix<float, 6, 1> Config::correction;
Eigen::Vector3f Config::distortion_corr;


template <typename T>
Eigen::Matrix<T,4,4> EigenIsoInv(const Eigen::Matrix<T,4,4> &Tcw) {
    Eigen::Matrix<T,3,3> Rcw = Tcw.block(0, 0, 3, 3);
    Eigen::Matrix<T,3,1> tcw = Tcw.block(0, 3, 3, 1);
    Eigen::Matrix<T,3,3> Rwc = Rcw.transpose();
    Eigen::Matrix<T,3,1> twc = -Rwc * tcw;

    Eigen::Matrix<T,4,4> Twc = Eigen::Matrix<T,4,4>::Identity();

    Twc.block(0, 0, 3, 3) = Rwc;
    Twc.block(0, 3, 3, 1) = twc;

    return Twc;
}

void Config::readConfig(){

    config_node_ = YAML::LoadFile(rgbd_localization::CONFIG_YAML_PATH);
    
    //constants
    x_min = config_node_["x_min"].as<float>();
    x_max = config_node_["x_max"].as<float>();
    y_min = config_node_["y_min"].as<float>();
    y_max = config_node_["y_max"].as<float>();
    z_min = config_node_["z_min"].as<float>();
// brightness filter
    brightnessStrictness = config_node_["brightnessStrictness"].as<float>();
// LED Separation
    RStrcitnessMax = config_node_["RStrcitnessMax"].as<float>();
    RStrictnessMin = config_node_["RStrictnessMin"].as<float>();
    GStrcitnessMax = config_node_["GStrcitnessMax"].as<float>();
    GStrictnessMin = config_node_["GStrictnessMin"].as<float>();
// noise removal
    min_valid_vel = config_node_["min_valid_vel"].as<float>();
    max_valid_y_vel = config_node_["max_valid_y_vel"].as<float>();
    min_LED_dis = config_node_["min_LED_dis"].as<float>();
    max_LED_dis = config_node_["max_LED_dis"].as<float>();
// finish criteria
    for(int i=0; i<3; i++)
        finishPos(i) = config_node_["finishPos"][i].as<float>();
    finishThreshold = config_node_["finishThreshold"].as<float>();
    
    // topics
    rgb_topic = config_node_["rgb_topic"].as<std::string>();
    depth_topic = config_node_["depth_topic"].as<std::string>();
    raw_cloud_topic = config_node_["raw_cloud_topic"].as<std::string>();
    calied_cloud_topic = config_node_["calied_cloud_topic"].as<std::string>();
    boat_cloud_topic = config_node_["boat_cloud_topic"].as<std::string>();
    red_cloud_topic = config_node_["red_cloud_topic"].as<std::string>();
    green_cloud_topic = config_node_["green_cloud_topic"].as<std::string>();
    localization_topic = config_node_["localization_topic"].as<std::string>();
    boat_vel_vis_topic = config_node_["boat_vel_vis_topic"].as<std::string>();
    to_control_topic = config_node_["to_control_topic"].as<std::string>();

    rgb_subpath = config_node_["rgb_subpath"].as<std::string>();
    depth_subpath = config_node_["depth_subpath"].as<std::string>();
    rosbag_subpath = config_node_["rosbag_subpath"].as<std::string>();

    for(size_t i = 0; i < 3; i++)
        for(size_t j = 0; j < 3; j++)
            R_MATLAB(i, j) = config_node_["R_MATLAB"][3 * i + j].as<float>();

    for(size_t i = 0; i < 3; i++)
        p_MATLAB(i) = config_node_["p_MATLAB"][i].as<float>();

    for(size_t i = 0; i < 6; i++)
        correction(i) = config_node_["correction"][i].as<float>();

    for(size_t i = 0; i < 3; i++)
        distortion_corr(i) = config_node_["distortion_corr"][i].as<float>();
}