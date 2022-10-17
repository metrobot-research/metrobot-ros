//
// Created by warren on 2021/10/22.
//

#include <ros/ros.h>
#include "glog/logging.h"
#include "global_definition/global_definition.h"
#include "subscriber/img_saver.hpp"
#include "localization_config.h"

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "img_save_node");
    ros::NodeHandle nh("~");

    Config::readConfig();

    bool screen = false;
    nh.param<bool>("screen", screen, "true");

    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = screen;
    FLAGS_colorlogtostderr = true;
    FLAGS_log_prefix = true;
    FLAGS_logbufsecs = 0;

    LOG(INFO) << "img_save_node/rgb_topic: " << Config::rgb_topic;
    LOG(INFO) << "img_save_node/depth_topic: " << Config::depth_topic;
    LOG(INFO) << "img_save_node/screen: " << screen;

//    Config::readConfig();

    std::shared_ptr<ImgSaver> rgb_save_ptr = std::make_shared<ImgSaver>(nh, Config::rgb_topic, 10, Config::rgb_subpath);
//    std::shared_ptr<ImgSaver> depth_save_ptr = std::make_shared<ImgSaver>(nh, depth_topic, 10, "depth");

    ros::Rate rate(30);
    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}