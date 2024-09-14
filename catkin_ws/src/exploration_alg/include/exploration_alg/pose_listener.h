#pragma once

#include <tf2_ros/transform_listener.h>

#include <string>

#include "ros/ros.h"

class PoseListener {
   public:
    PoseListener(ros::NodeHandle* node);
    void getRobotWorldPosition(double& wx, double& wy, double& wtheta);

   private:
    ros::NodeHandle* node_;
    std::string global_frame_;
    std::string base_frame_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};