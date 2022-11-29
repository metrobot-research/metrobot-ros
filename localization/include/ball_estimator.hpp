//
// Created by warren on 2022/11/27.
//

#ifndef CATKIN_CHICKEN_ROBOT_BALL_ESTIMATOR_H
#define CATKIN_CHICKEN_ROBOT_BALL_ESTIMATOR_H

#include <iostream>
#include <cassert>
#include <deque>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct BallPosStamped{
    Eigen::Vector3f position;
    ros::Time timestamp;
};

class BallEstimator{
public:
    BallEstimator(ros::NodeHandle &nh): // params not set to const since we may need real-time tuning
            nh_(nh), give_up(true), lost_time(0), has_inited(false), step_time(0), window_time(0),
            cur_ball_vel_w(0,0,0){

        nh_.getParam("ball_est_pred_mode", pred_mode);
        nh_.getParam("ball_est_max_window_time", max_window_time);
        nh_.getParam("ball_est_max_lost_time", max_lost_time);
        nh_.getParam("ball_est_bounce_lost_coeff", bounce_lost_coeff);
        float freq;
        nh_.getParam("camera2_color_fps", freq);
        frame_dt = 1. / freq;

        ball_pos_stamped_his.clear();
    };

    // Add Measurement and then Update Estimation
    void addPos(const ros::Time &timestamp, const Eigen::Vector3f &new_ball_pos_w, const Eigen::Vector3f &new_wheel_center_w){
        if(give_up){
            give_up = false;
            std::cout << "Redetected ball-----------" << std::endl;
        }
        lost_time = 0;

        if(has_inited)
            step_time = (timestamp - last_time).toSec();
        last_time = timestamp;

        BallPosStamped newPosStamped = {new_ball_pos_w, timestamp};
        ball_pos_stamped_his.push_back(newPosStamped);
        window_time = (ball_pos_stamped_his.back().timestamp - ball_pos_stamped_his.front().timestamp).toSec();
        while(window_time > max_window_time){
            ball_pos_stamped_his.pop_front();
            window_time = (ball_pos_stamped_his.back().timestamp - ball_pos_stamped_his.front().timestamp).toSec();
        }

        cur_wheel_center_w = new_wheel_center_w;
        estimateCurBallState();

        has_inited = true;
    };

    void addLost(const ros::Time &timestamp, const Eigen::Vector3f &new_wheel_center_w){
        if(!has_inited){
//            std::cout << "Give up-------------" << std::endl;
            return;
        }
        step_time = (timestamp - last_time).toSec();
        last_time = timestamp;
        if(!ball_pos_stamped_his.empty()){
            window_time = (timestamp - ball_pos_stamped_his.front().timestamp).toSec();
            while(window_time > max_window_time && !ball_pos_stamped_his.empty()){
                ball_pos_stamped_his.pop_front();
                window_time = (timestamp - ball_pos_stamped_his.front().timestamp).toSec();
            }
        }else
            window_time = 0;

        if(lost_time < max_lost_time){
            lost_time += step_time;
            if(lost_time >= max_lost_time){
                give_up = true;
//                std::cout << "Give up-------------" << std::endl;
            }else{
                cur_wheel_center_w = new_wheel_center_w;
                estimateCurBallState();
            }
        }else{
//            std::cout << "Give up-------------" << std::endl;
        }

    };

    // Get Estimation Result
    bool isGiveup(){return give_up;};
    Eigen::Vector3f getCurBallPos(){return cur_ball_pos_w;}; // always check isLost before querying for cur ball pos & vel
    Eigen::Vector3f getCurBallVel(){return cur_ball_vel_w;};

private:
    void estimateCurBallState(){
        if(!give_up){
            if(lost_time > 0){ // lost but not give up, do pred
                updateBallPred(); // always has prev ball state if give_up = false && ball_position_his.empty()
                                  // the only case leading to this is lost for cur frame, but has previous estimate
            }else if(window_time < max_window_time / 2){ // moving window not long enough, calc vel will introduce noise and spike
                cur_ball_pos_w = ball_pos_stamped_his.back().position;
                cur_ball_vel_w.setZero(); //fixme: use pred.ed spd here, or spd will be reset to 0 too quickly
            }else{ // moving window long enough, apply moving window vel filter(smooth out differentiation noise)
                cur_ball_pos_w = ball_pos_stamped_his.back().position;
                cur_ball_vel_w = (ball_pos_stamped_his.back().position - ball_pos_stamped_his.front().position) / window_time;
            }

            prev_ball_pos_w = cur_ball_pos_w;
            prev_ball_vel_w = cur_ball_vel_w;
            prev_ball_height = cur_ball_pos_w.z() - cur_wheel_center_w.z();
        }
//        std::cout << "cur ball vel: " << cur_ball_vel_w << std::endl;
    };

    void updateBallPred(){
        if(pred_mode == "uniform"){
            cur_ball_pos_w = prev_ball_pos_w + step_time * prev_ball_vel_w;
            cur_ball_vel_w = prev_ball_vel_w;
        }else if(pred_mode == "gravity"){
            // Assume uniform velocity in x,y
            cur_ball_pos_w.block(0,0,2,1) = prev_ball_pos_w.block(0,0,2,1)
                                            + step_time * prev_ball_vel_w.block(0,0,2,1);
            cur_ball_vel_w.block(0,0,2,1) = prev_ball_vel_w.block(0,0,2,1);
            // z pos & vel pred is different whether previously on gnd
            if(prev_ball_height < 0.1 && abs(prev_ball_vel_w.z()) < 0.05){ // previously on gnd, with tolerance in pos & vel meas
                cur_ball_vel_w.z() = 0;
                cur_ball_pos_w.z() = cur_wheel_center_w.z();
            }else{ // previously above gnd
                float t_hit_gnd = (prev_ball_vel_w.z() + sqrt(prev_ball_vel_w.z()*prev_ball_vel_w.z() + 2*9.8*prev_ball_height)) / 9.8;
                if(t_hit_gnd > step_time){ // ball does not hit gnd
                    cur_ball_pos_w.z() = prev_ball_pos_w.z() + step_time * prev_ball_vel_w.z() - 0.5 * step_time * step_time * 9.8; // assume acc is g
                    cur_ball_vel_w.z() = prev_ball_vel_w.z() - step_time * 9.8;
                }else{ // ball hits gnd, and bounce back
                    float v_bounce_back = -bounce_lost_coeff * (prev_ball_vel_w.z() - t_hit_gnd * 9.8);
                    float rise_time = step_time - t_hit_gnd;
                    if(rise_time > 2 * v_bounce_back / 9.8){
                        // will bounce again. indicating z vel is already small enough, simplify calc by setting z vel to 0, and ball to gnd
                        cur_ball_vel_w.z() = 0;
                        cur_ball_pos_w.z() = cur_wheel_center_w.z();
                    }else{ // large bouncing back vel, won't bounce again
                        cur_ball_vel_w.z() = v_bounce_back - 9.8 * rise_time;
                        cur_ball_pos_w.z() = cur_wheel_center_w.z() + v_bounce_back * rise_time - 0.5 * rise_time * rise_time * 9.8;
                    }

                }
            }
        }else
            std::cerr << "Invalid pred mode in ball_estimator (valid opts: uniform, gravity)" << std::endl;
        
    };
    
    
    ros::NodeHandle nh_;
    std::string pred_mode;
    float max_window_time;
    float max_lost_time;
    float bounce_lost_coeff;

    float lost_time;
    bool give_up;
    bool has_inited;
    ros::Time last_time;
    float window_time;
    float frame_dt; // default frame dt specified in launch
    float step_time; // dt between the cur and prev measurement (lost is also a measurement)
    std::deque<BallPosStamped> ball_pos_stamped_his;
    Eigen::Vector3f prev_ball_pos_w;
    Eigen::Vector3f prev_ball_vel_w;
    float prev_ball_height;
    Eigen::Vector3f cur_ball_pos_w;
    Eigen::Vector3f cur_ball_vel_w;
    Eigen::Vector3f cur_wheel_center_w;
};

#endif //CATKIN_CHICKEN_ROBOT_BALL_ESTIMATOR_H
