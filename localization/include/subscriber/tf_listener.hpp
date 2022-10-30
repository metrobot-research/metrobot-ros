#ifndef SUBSCRIBER_TF_LISTENER_HPP
#define SUBSCRIBER_TF_LISTENER_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

class TFListener{
public:
    TFListener() = default;

    //// --------------------lookup latest transform------------------------
    void lookupTransform(const std::string &parent_frame, const std::string child_frame, Eigen::Vector3f &p, Eigen::Quaternionf &q){
        tf_listener_.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
        p = Eigen::Vector3f(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
        q = Eigen::Quaternionf(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
    };

    void lookupTransformMatrix(const std::string &parent_frame, const std::string child_frame, Eigen::Matrix4f &T){
        Eigen::Vector3f p;
        Eigen::Quaternionf q;
        lookupTransform(parent_frame, child_frame,p,q);
        T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = q.toRotationMatrix();
        T.block<3,1>(0,3) = p;
    };

    //// --------------------lookup transform at specified time------------------------
//    void lookupTransform(const std::string &parent_frame, const std::string child_frame, ros::Time time, Eigen::Vector3f &p, Eigen::Quaternionf &q){
//        tf_listener_.lookupTransform(parent_frame, child_frame, time, transform);
//        p = Eigen::Vector3f(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
//        q = Eigen::Quaternionf(transform.getRotation().w(), transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z());
//    };
//
//    void lookupTransformMatrix(const std::string &parent_frame, const std::string child_frame, ros::Time time, Eigen::Matrix4f &T){
//        Eigen::Vector3f p;
//        Eigen::Quaternionf q;
//        lookupTransform(parent_frame, child_frame, time,p,q);
//        T = Eigen::Matrix4f::Identity();
//        T.block<3,3>(0,0) = q.toRotationMatrix();
//        T.block<3,1>(0,3) = p;
//    };

private:

    tf::TransformListener tf_listener_;
    tf::StampedTransform transform;
};

#endif //SUBSCRIBER_TF_LISTENER_HPP
