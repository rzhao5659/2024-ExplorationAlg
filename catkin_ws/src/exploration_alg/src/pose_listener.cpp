#include "pose_listener.h"

#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <tf2_ros/transform_listener.h>

#include <string>

#include "ros/ros.h"

PoseListener::PoseListener(ros::NodeHandle* node) : node_(node), tf_buffer_(), tf_listener_(tf_buffer_) {
    // Get parameter values from parameter server. If not found, set to default values.
    node->param<std::string>("global_frame", this->global_frame_, "map");         // Map frame id
    node->param<std::string>("base_frame", this->base_frame_, "base_footprint");  // Robot frame id.
}

void PoseListener::getRobotWorldPosition(double& wx, double& wy, double& wtheta) {
    geometry_msgs::TransformStamped transformStamped;
    while (this->node_->ok()) {
        try {
            transformStamped = this->tf_buffer_.lookupTransform(this->global_frame_, this->base_frame_, ros::Time(0));
        } catch (tf2::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        break;
    }
    wx = transformStamped.transform.translation.x;
    wy = transformStamped.transform.translation.y;
    // Extract yaw from quaternion. Assuming 2D rotations.
    double qw = transformStamped.transform.rotation.w;  // cos(theta/2)
    double qz = transformStamped.transform.rotation.z;  // sin(theta/2)
    wtheta = 2 * atan2(qz, qw);
}