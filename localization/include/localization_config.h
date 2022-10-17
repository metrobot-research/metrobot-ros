//
// Created by qzj on 2021/4/25.
//

#ifndef LOCALIZATION_CONFIG_H
#define LOCALIZATION_CONFIG_H

#include <yaml-cpp/yaml.h>
#include "global_definition/global_definition.h"
#include <Eigen/Core>
#include <vector>
#include <Eigen/Geometry>
#include <iostream>
#include <fstream>      // std::ifstream
#include <stdio.h>
#include <unistd.h>

class Config {

public:

    Config(){}
    static void readConfig();

public:
    // constants
    // box filter
    static float x_min;
    static float x_max;
    static float y_min;
    static float y_max;
    static float z_min;
    // brightness filter
    static float brightnessStrictness;
    // LED Separation
    static float RStrcitnessMax;
    static float RStrictnessMin;
    static float GStrcitnessMax;
    static float GStrictnessMin;
    // noise removal
    static float min_valid_vel;
    static float max_valid_y_vel;
    static float min_LED_dis;
    static float max_LED_dis;
    // finish criteria
    static Eigen::Vector3f finishPos;
    static float finishThreshold;

    // topics
    static std::string rgb_topic;
    static std::string depth_topic;
    static std::string raw_cloud_topic;
    static std::string calied_cloud_topic;
    static std::string boat_cloud_topic;
    static std::string red_cloud_topic;
    static std::string green_cloud_topic;
    static std::string localization_topic;
    static std::string boat_vel_vis_topic;
    static std::string to_control_topic;
    static std::string rgb_subpath;
    static std::string depth_subpath;
    static std::string rosbag_subpath;

    static Eigen::Matrix3f R_MATLAB;
    static Eigen::Vector3f p_MATLAB;
    static Eigen::Matrix<float, 6, 1> correction;
    static Eigen::Vector3f distortion_corr;

    static YAML::Node config_node_;
};


#endif //LOCALIZATION_CONFIG_H
