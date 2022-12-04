//
// Created by warren on 2022/12/4.
//

#ifndef CATKIN_CHICKEN_ROBOT_PID_HPP
#define CATKIN_CHICKEN_ROBOT_PID_HPP

#include <iostream>
#include <cassert>
#include <deque>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

class PID{
public:
    PID(const ros::NodeHandle &nh, const std::string &name)
        : nh_(nh), error_integral(0), error_diff(0), has_inited(false){
        nh_.getParam(name + "_kp", kp_);
        nh_.getParam(name + "_ki", ki_);
        nh_.getParam(name + "kd", kd_);
        nh_.getParam(name + "_kw", kw_);
    };

    float generateCmd(ros::Time timestamp, float error){
        if(has_inited){
            float dt = (timestamp - last_timestamp).toSec();
            error_integral = error_integral * kw_ + error * dt;
            error_diff = (error - last_error) / dt;
        }else
            has_inited = true;

        last_timestamp = timestamp;
        last_error = error;
        return kp_ * error + ki_ * error_integral + kd_ * error_diff;
    };

private:
    ros::NodeHandle nh_;
    float kp_, ki_, kd_, kw_;
    float error_integral, error_diff;
    bool has_inited;
    ros::Time last_timestamp;
    float last_error;
};


#endif //CATKIN_CHICKEN_ROBOT_PID_HPP
