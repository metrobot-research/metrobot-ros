#ifndef SRC_LOCALIZATION_FLOW_H
#define SRC_LOCALIZATION_FLOW_H

#include <ros/ros.h>
// subscriber
#include "subscriber/dual_img_subscriber.hpp"
#include "subscriber/tf_listener.hpp"
// publisher
#include "publisher/cloud_publisher.hpp"
#include "publisher/arrow_publisher.hpp"
#include "publisher/tf_broadcaster.hpp"
// sensor data
#include <sensor_msgs/CameraInfo.h>
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/dual_img_stamped.hpp"
// ball estimator
#include "ball_estimator.hpp"
// yaml
#include <yaml-cpp/yaml.h>
// tools
#include "tools/tic_toc.hpp"

using namespace cv;
using namespace std;

class LocalizationFlow{
public:
    LocalizationFlow(ros::NodeHandle &nh);

    void Run();

    // for rviz
    void GenerateFullPointCloud();

    // for use
    void updateParams();
    void SegmentBall2D();
    void GetBallCloud();
    void CalcBallCenter3D();

    static void on_trackbar( int, void* ){
        //This function gets called whenever a
        // trackbar position is changed
    };
    std::string intToString(int number);
    void createTrackbars();
    void drawObject(int x, int y, Mat &frame);
    void morphOps(Mat &thresh);
    void trackFilteredObject(int &x, int &y, Mat threshold, Mat &cameraFeed);

private:
    // node handle
    ros::NodeHandle nh_;
    // cam calibration
    Eigen::Matrix3f K_inv;
    // subscriber
    std::shared_ptr<DualImgSubscriber> rgb_d_sub_ptr_;
    std::shared_ptr<TFListener> tf_listener_ptr_;
    // publisher
    //   for use
    std::shared_ptr<TFBroadCaster> tf_broadcast_ptr_;
    //   for rviz
    std::shared_ptr<CloudPublisher> full_cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> ball_cloud_pub_ptr_;
    std::shared_ptr<ArrowPublisher> ball_vel_pub_ptr_;
    // ball estimator
    BallEstimator ball_estimator;

    // params
    // visualization opt
    bool cv_vis;
    bool check_point_cloud;
    // camera params
    float clip_z_dis[2]; // min, max in meters, according to datasheet
    // ball property
    float ball_real_radius = 0.033;
    //  frame size
    int FRAME_WIDTH = 640;
    int FRAME_HEIGHT = 480;
    //  ball thresholding
    int H_MIN = 0;
    int H_MAX = 256;
    int S_MIN = 0;
    int S_MAX = 256;
    int V_MIN = 0;
    int V_MAX = 256;
    //  object filtering
    bool trackObjects = true;
    bool useMorphOps = true;
    int ERODE_DIAM = 3;
    int DILATE_DIAM = 8;
    int MIN_OBJ_PIX_DIAM = 20;
    int MIN_OBJECT_AREA = MIN_OBJ_PIX_DIAM * MIN_OBJ_PIX_DIAM;
    int MAX_OBJ_PERCENTAGE = 70;
    int MAX_OBJECT_AREA = FRAME_HEIGHT * FRAME_WIDTH * MAX_OBJ_PERCENTAGE / 100.;
    int MAX_NUM_OBJECTS=50; //max number of objects to be detected in frame

    //names that will appear at the top of each window
    const string windowName = "Original Image";
    const string windowName1 = "HSV Image";
    const string windowName2 = "Thresholded Image";
    const string windowName3 = "After Morphological Operations";
    const string trackbarWindowName = "Trackbars";

    // data processing flow
    // read in bgr & depth imgs, and wheel_center
    std::deque<DualImgStamped> rgb_d_buffer_;
    DualImgStamped cur_rgbd_stamped;
    Eigen::Vector3f cur_d435i_pos;
    Eigen::Quaternionf cur_d435i_ori;
    std::deque<Eigen::Vector3f> wheel_center_buffer_;
    Eigen::Vector3f cur_wheel_center;
    // SegmentBall2D
    bool found_ball;
    int ball_i2D, ball_j2D;
    int ball_pix_radius;
    cv::Mat imgThresed;
    // GetBallCenter3D
    pcl::PointCloud<pcl::PointXYZ>::Ptr ball_cloud_ptr;
    Eigen::Vector3f ball_center;

    // for rviz
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr full_cloud_ptr;

    // timing
    std::shared_ptr<TicToc> time_run;
};

#endif //SRC_LOCALIZATION_FLOW_H