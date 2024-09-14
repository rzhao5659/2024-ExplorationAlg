#pragma once
#include <memory>

#include "map.h"
#include "pose_listener.h"
#include "ros/ros.h"

// This class is mainly to compute the gain / score of a given point (wx,wy).
class GainEvaluator {
   public:
    void computeGain(double wx, double wy, double& wtheta, double& score);
    void computeGain(double goal_wx, double goal_wy, double& goal_wtheta, double& goal_score, std::string& info);
    GainEvaluator(ros::NodeHandle* node, std::shared_ptr<Map2D> map, PoseListener* pose_listener);

   private:
    PoseListener* pose_listener_;
    std::shared_ptr<Map2D> map_;
    std::vector<std::vector<double>> rays_;  // precomputed rays (unit vectors) across 360 degrees according to raytrace ang resolution.
    double sensor_max_range_;                // [m].  Used for sampling radius.
    double sensor_fov_[2];                   // [rad]. fov min (must be negative) and max (must be positive) w.r.t robot's forward direction.  Limited to [-180, 180].
    double raytrace_ang_resolution_;         // [degrees].
    double raytrace_lin_resolution_;         // [m]
    double vmax_;
    double wmax_;
    bool visualize_gain_;
    ros::Publisher rviz_marker_pub_;
};