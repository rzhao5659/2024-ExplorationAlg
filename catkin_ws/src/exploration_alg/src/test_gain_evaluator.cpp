#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <vector>

#include "cell2d.h"
#include "gain_evaluator.h"
#include "map.h"
#include "map_client.h"
#include "pose_listener.h"
#include "ros/ros.h"

// Copies gain evaluator but with rviz visualization code.

inline double normalizeAngle(int ang) {
    ang %= 360;
    if (ang < 0) {
        ang += 360;
    }
    return ang;
}

inline double distance(double x1, double y1, double x2, double y2) { return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)); }

GainEvaluator::GainEvaluator(ros::NodeHandle* node, std::shared_ptr<Map2D> map, PoseListener* pose_listener) : map_(map), pose_listener_(pose_listener) {
    node->param<double>("raytrace_lin_resolution", raytrace_lin_resolution_, 0.05);
    node->param<double>("raytrace_ang_resolution", raytrace_ang_resolution_, 2.0);
    node->param<double>("sensor_max_range", sensor_max_range_, 3.50);
    node->param<double>("sensor_fov_lower", sensor_fov_[0], -80);
    node->param<double>("sensor_fov_upper", sensor_fov_[1], 80);
    node->param<double>("max_linear_vel", vmax_, 0.22);
    node->param<double>("max_angular_vel", wmax_, 2.84);

    // Round down angular resolution so that it's divisible by 360.
    int num_rays = floor(360 / raytrace_ang_resolution_);
    raytrace_ang_resolution_ = 360 / num_rays;

    // Precompute rays unit vectors.
    rays_.resize(num_rays);
    for (int i = 0; i < num_rays; i++) {
        double angle = (raytrace_ang_resolution_ * i) * M_PI / 180;
        rays_[i].resize(2);
        rays_[i][0] = cos(angle);
        rays_[i][1] = sin(angle);
    }

    // RVIZ
    rviz_marker_pub_ = node->advertise<visualization_msgs::MarkerArray>("/explore/rviz/gain", 1);
}

void GainEvaluator::computeGain(double goal_wx, double goal_wy, double& goal_wtheta, double& goal_score) {
    int num_points_along_ray = floor(sensor_max_range_ / raytrace_lin_resolution_);  // without considering initial point from each ray.
    std::vector<double> information_gain_rays;                                       // This will store information gain along each ray.
    information_gain_rays.resize(rays_.size());
    std::unordered_set<cell2d_t, cell2d_t::hash> counted_set;  // doing raytrace might encounter same unknown cell, don't want to count twice.
    auto notCounted = [&](cell2d_t& cell) { return counted_set.find(cell) == counted_set.end(); };

    // print current robot position.
    double robot_wx1, robot_wy1, robot_wtheta1;
    this->pose_listener_->getRobotWorldPosition(robot_wx1, robot_wy1, robot_wtheta1);
    ROS_DEBUG("ROBOT CURRENT POSITION is (%.3f,%.3f,%.3f)", robot_wx1, robot_wy1, robot_wtheta1 * 180 / M_PI);

    // RVIZ
    visualization_msgs::MarkerArray markers_msg;
    std::vector<visualization_msgs::Marker>& markers = markers_msg.markers;
    // Common header for all markers.
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();
    visualization_msgs::Marker points, point, arrow;
    points.header = header;
    points.ns = "unknown_cells";
    points.type = visualization_msgs::Marker::POINTS;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.05;  // x and y scale width/height of point.
    points.scale.y = 0.05;
    points.color.r = 1.0;  // red points.
    points.color.a = 1.0;
    points.lifetime = ros::Duration();  // forever.
    points.frame_locked = true;
    point.header = header;
    point.ns = "position";
    point.type = visualization_msgs::Marker::SPHERE;
    point.pose.position.x = goal_wx;
    point.pose.position.y = goal_wy;
    point.pose.orientation.w = 1.0;
    point.scale.x = 0.3;
    point.scale.y = 0.3;
    point.scale.z = 0.3;
    point.color.g = 1.0;  // red point.
    point.color.a = 1.0;
    point.lifetime = ros::Duration();  // forever.
    point.frame_locked = true;

    // Compute information gain for each ray, by raytracing and count number of unknown cells, then getting the total length of unknown space.
    for (int i = 0; i < rays_.size(); i++) {
        int num_unknowns = 0;
        counted_set.clear();
        for (int j = 0; j < num_points_along_ray; j++) {
            double length = (j + 1) * raytrace_lin_resolution_;
            double wx = length * rays_[i][0] + goal_wx;
            double wy = length * rays_[i][1] + goal_wy;

            // Get corresponding cell in map.
            cell2d_t cell;
            bool withinBound = map_->worldToMap(wx, wy, cell.x, cell.y);
            if (!withinBound) {
                // stop raytrace.
                break;
            }

            // Check corresponding cell type.
            int idx = map_->getIndex(cell.x, cell.y);
            int cell_state = map_->map_data[idx].occupancy_state;
            ROS_DEBUG("Evaluating point(%.3f, %.3f) with state %d", wx, wy, cell_state);
            if (cell_state == UNKNOWN_SPACE && notCounted(cell)) {
                num_unknowns++;
                counted_set.insert(cell);
                // RVIZ
                geometry_msgs::Point p;
                p.x = wx;
                p.y = wy;
                points.points.push_back(p);
            } else if (cell_state == OCCUPIED_SPACE) {
                // Stop counting as ray stops there.
                break;
            } else {
                // free space: do nothing and continue raytrace.
            }
        }

        // Store information gain for each ray as total length of unknown cells.
        information_gain_rays[i] = num_unknowns * map_->resolution;
        ROS_DEBUG("ray[%d] with dir(%.3f, %.3f) has number of unknown cells %d", i, rays_[i][0], rays_[i][1], num_unknowns);
    }

    // RVIZ
    markers.push_back(points);
    markers.push_back(point);

    // Now that we have information gain for each ray, optimize yaw.
    double best_information_gain = -1;

    for (int i = 0; i < rays_.size(); i++) {
        int theta = (int)i * raytrace_ang_resolution_;
        // Compute information gain at yaw theta by summing up length of unknown cells in that yaw within sensor fov.
        double information_gain = 0.0;
        int theta_min = normalizeAngle(theta + sensor_fov_[0]);
        int theta_max = normalizeAngle(theta + sensor_fov_[1]);
        // Get the ray index for theta_min and theta_max
        int theta_min_ray_idx = floor(theta_min / raytrace_ang_resolution_);
        int theta_max_ray_idx = floor(theta_max / raytrace_ang_resolution_);
        // Sum length of unknown cells from min idx to max idx.
        int idx = theta_min_ray_idx;
        while (idx != theta_max_ray_idx) {
            information_gain += information_gain_rays[idx];
            idx = (idx + 1) % rays_.size();
        }
        information_gain += information_gain_rays[theta_max_ray_idx];

        if (information_gain > best_information_gain) {
            best_information_gain = information_gain;
            goal_wtheta = theta;
        }
    }
    goal_wtheta *= M_PI / 180;

    // Compute time to reach. Assumes robot is driving at vmax and wmax, and a collision-free straight path.
    double robot_wx, robot_wy, robot_wtheta;
    pose_listener_->getRobotWorldPosition(robot_wx, robot_wy, robot_wtheta);
    double heading_diff = std::min(abs(goal_wtheta - robot_wtheta), 2 * M_PI - abs(goal_wtheta - robot_wtheta));
    double heading_time_to_reach = heading_diff / wmax_;
    double position_time_to_reach = distance(goal_wx, goal_wy, robot_wx, robot_wy) / vmax_;
    double time_to_reach = std::max(heading_time_to_reach, position_time_to_reach);

    // The total gain for this pose will be information gain / time to reach that pose.
    goal_score = best_information_gain / time_to_reach;

    ROS_DEBUG("Optimized yaw:%.3f Information gain:%.3f Time to reach:%.3f Score:%.3f ", goal_wtheta * 180 / M_PI, best_information_gain, time_to_reach, goal_score);

    // RVIZ
    arrow.header = header;
    arrow.ns = "optimal yaw";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.pose.position.x = goal_wx;
    arrow.pose.position.y = goal_wy;
    arrow.pose.orientation.w = cos(goal_wtheta / 2);
    arrow.pose.orientation.z = sin(goal_wtheta / 2);
    arrow.scale.x = 0.3;
    arrow.scale.y = 0.1;
    arrow.scale.z = 0.1;
    arrow.color.g = 1.0;  // g arrow.
    arrow.color.a = 1.0;
    arrow.lifetime = ros::Duration();  // forever.
    arrow.frame_locked = true;
    markers.push_back(arrow);
    rviz_marker_pub_.publish(markers_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_gain_node");
    ros::NodeHandle node;
    PoseListener pose_listener(&node);
    Map2DClient map_client(node, pose_listener);
    GainEvaluator gain_evaluator(&node, map_client.getMap(), &pose_listener);

    while (ros::ok()) {
        ROS_INFO("Input a real world position to evaluate its gain.");
        std::cin.clear();
        double wx, wy;
        std::string operation;
        std::cin >> operation;
        if (operation != "compute") {
            ros::spinOnce();
            continue;
        }

        std::cin >> wx >> wy;
        ROS_INFO("Received %.3f, %.3f", wx, wy);
        double wtheta, score;
        gain_evaluator.computeGain(wx, wy, wtheta, score);
        ros::spinOnce();
    }

    return 0;
}
