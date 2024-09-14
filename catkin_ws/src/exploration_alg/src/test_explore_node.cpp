#include <visualization_msgs/MarkerArray.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "exploration_alg/ExplorationGoalRequest.h"
#include "explore_node.h"
#include "frontier_detector.h"
#include "map_client.h"
#include "ros/ros.h"
#include "std_msgs/Header.h"

using namespace exploration_alg;

ExploreNode::ExploreNode()
    : node_(),
      pose_listener_(&node_),
      map_client_(node_, pose_listener_),
      frontier_detector_(node_, map_client_.getMap(), &pose_listener_),
      gain_evaluator_(&node_, map_client_.getMap(), &pose_listener_) {
    this->node_.param<int>("min_frontier_size", this->min_frontier_size_, 8);

    this->node_.param<bool>("visualize", visualize_, true);
    this->node_.param<bool>("publish_map", publish_map_, false);  // useful for testing, otherwise don't
    this->node_.param<std::string>("global_frame", this->global_frame_, "map");
    this->node_.param<double>("sampling_radius", this->sampling_radius_, 1.0);

    this->exploration_goal_srv_ = this->node_.advertiseService("/explore/get_goal", &ExploreNode::getExplorationGoal, this);
    this->is_exploration_done_ = true;  // this is just the default value.
    this->exploration_goal_score_ = -1;

    if (visualize_) {
        this->rviz_marker_pub_ = this->node_.advertise<visualization_msgs::MarkerArray>("/explore/rviz/frontiers", 1);
        this->rviz_pub_timer_ = this->node_.createTimer(ros::Duration(0.1), std::bind(&ExploreNode::visualizeCallback, this));
    }
    if (publish_map_) {
        this->map_pub_ = this->node_.advertise<nav_msgs::OccupancyGrid>("/explore/map/occupied", 10);
        this->visited_pub_ = this->node_.advertise<nav_msgs::OccupancyGrid>("/explore/map/visited", 10);
        this->map_pub_timer_ = this->node_.createTimer(ros::Duration(1.0), std::bind(&ExploreNode::publishMap, this));
    }

    ROS_INFO("ExploreNode initialized");
}

void ExploreNode::publishMap() {
    nav_msgs::OccupancyGrid occupancy_grid_msg;
    occupancy_grid_msg.header.stamp = ros::Time::now();
    occupancy_grid_msg.header.frame_id = "map";
    auto map = this->map_client_.getMap();
    occupancy_grid_msg.info.resolution = map->resolution;        // meters per pixel
    occupancy_grid_msg.info.width = map->size_x;                 // number of cells in x direction
    occupancy_grid_msg.info.height = map->size_y;                // number of cells in y direction
    occupancy_grid_msg.info.origin.position.x = map->origin_wx;  // origin of the map in meters
    occupancy_grid_msg.info.origin.position.y = map->origin_wy;
    occupancy_grid_msg.info.origin.position.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.x = 0.0;
    occupancy_grid_msg.info.origin.orientation.y = 0.0;
    occupancy_grid_msg.info.origin.orientation.z = 0.0;
    occupancy_grid_msg.info.origin.orientation.w = 1.0;
    // Occupancy state data.
    occupancy_grid_msg.data.resize(occupancy_grid_msg.info.width * occupancy_grid_msg.info.height);
    for (int i = 0; i < map->size_x * map->size_y; i++) {
        occupancy_grid_msg.data[i] = map->map_data[i].occupancy_state;
    }
    this->map_pub_.publish(occupancy_grid_msg);
    // Visited state data
    for (int i = 0; i < map->size_x * map->size_y; i++) {
        occupancy_grid_msg.data[i] = map->map_data[i].visited_state == true ? 100 : -1;
    }
    this->visited_pub_.publish(occupancy_grid_msg);
}

void ExploreNode::sampleFreePointsAroundPoint(double wx, double wy, int n, double radius, std::vector<point2d_t>& sampled_free_points) {
    // Scan through an area in map that encloses the circle with this radius, and store the free cells coordinates.
    std::shared_ptr<Map2D> map = this->map_client_.getMap();
    std::vector<cell2d_t> free_cells_in_area;
    double bottom_left_corner_wx = wx - radius;
    double bottom_left_corner_wy = wy - radius;
    double top_right_corner_wx = wx + radius;
    double top_right_corner_wy = wy + radius;
    int bottom_left_corner_gx, bottom_left_corner_gy, top_right_corner_gx, top_right_corner_gy;
    map->worldToMap(bottom_left_corner_wx, bottom_left_corner_wy, bottom_left_corner_gx, bottom_left_corner_gy);
    map->worldToMap(top_right_corner_wx, top_right_corner_wy, top_right_corner_gx, top_right_corner_gy);

    for (int gx = bottom_left_corner_gx; gx <= top_right_corner_gx && gx < map->size_x; gx++) {
        for (int gy = bottom_left_corner_gy; gy <= top_right_corner_gy && gy < map->size_y; gy++) {
            int idx = map->getIndex(gx, gy);
            if (map->map_data[idx].occupancy_state == FREE_SPACE) {
                cell2d_t free_cell = {gx, gy};
                free_cells_in_area.emplace_back(free_cell);
            }
        }
    }

    if (n > free_cells_in_area.size()) {
        for (cell2d_t& cell : free_cells_in_area) {
            // Use each grid cell as sampled points.
            point2d_t free_cell_W;
            map->mapToWorld(cell.x, cell.y, free_cell_W.x, free_cell_W.y);
            sampled_free_points.emplace_back(free_cell_W);
        }
    } else {
        for (int i = 0; i < n; i++) {
            // Sample randomly in these free cells.
            int idx = rand() % free_cells_in_area.size();
            cell2d_t free_cell_M = free_cells_in_area[idx];
            point2d_t free_cell_W;
            map->mapToWorld(free_cell_M.x, free_cell_M.y, free_cell_W.x, free_cell_W.y);
            sampled_free_points.emplace_back(free_cell_W);
        }
    }
}

bool ExploreNode::getExplorationGoal(ExplorationGoalRequest::Request& req, ExplorationGoalRequest::Response& res) {
    ROS_INFO("Explore Node received an exploration goal request.");

    std::shared_ptr<Map2D> map = this->map_client_.getMap();

    // Filter out small size frontiers.
    std::vector<Frontier*> filtered_frontiers;
    for (Frontier& frontier : this->frontier_detector_.frontiers) {
        if (frontier.size >= this->min_frontier_size_) {
            filtered_frontiers.emplace_back(&frontier);
        }
    }

    // Check whether there are any frontiers left.
    if (filtered_frontiers.size() == 0) {
        res.is_exploration_done = true;
        this->is_exploration_done_ = true;
        ROS_INFO("Exploration is completed: No more exploration goal was found.");
        return true;
    }
    this->is_exploration_done_ = false;

    double best_score = -1;
    double best_exploration_goal[3];  // x,y,theta
    best_goal_per_frontier_.resize(filtered_frontiers.size());
    int i = 0;

    for (Frontier* frontier : filtered_frontiers) {
        double best_score_frontier = -1.0;
        best_goal_per_frontier_[i].resize(3);

        // Sample free cells around each frontier centroid, and score them all.
        std::vector<point2d_t> sampled_points;
        std::vector<double> sampled_scores;  // DELETE LATER
        sampleFreePointsAroundPoint(frontier->centroid[0], frontier->centroid[1], 200, this->sampling_radius_, sampled_points);

        for (point2d_t& point : sampled_points) {
            double point_optimal_yaw, point_score;
            this->gain_evaluator_.computeGain(point.x, point.y, point_optimal_yaw, point_score);
            sampled_scores.push_back(point_score);  // DELETE LATER.
            if (point_score > best_score) {
                best_score = point_score;
                best_exploration_goal[0] = point.x;
                best_exploration_goal[1] = point.y;
                best_exploration_goal[2] = point_optimal_yaw;
            }

            if (point_score > best_score_frontier) {
                best_score_frontier = point_score;
                best_goal_per_frontier_[i][0] = point.x;
                best_goal_per_frontier_[i][1] = point.y;
                best_goal_per_frontier_[i][2] = point_optimal_yaw;
            }
        }

        ROS_DEBUG("Best pose near Frontier[%d] is Pose(%.3f, %.3f, %.3f) with score %.3f", i, best_goal_per_frontier_[i][0], best_goal_per_frontier_[i][1], best_goal_per_frontier_[i][2] * 180 / M_PI,
                  best_score_frontier);
        // i++;  ADD THIS BACK LATER.

        // TEMPORAL
        std::vector<visualization_msgs::Marker>& markers = this->markers_msg_.markers;
        std_msgs::Header header;
        header.frame_id = this->global_frame_;
        header.stamp = ros::Time::now();
        visualization_msgs::Marker points;
        points.header = header;
        points.ns = "sampled_points";
        points.type = visualization_msgs::Marker::POINTS;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.05;  // x and y scale width/height of point.
        points.scale.y = 0.05;
        points.color.r = 1.0;
        points.color.a = 1.0;
        points.lifetime = ros::Duration();  // forever.
        points.id = i;
        points.frame_locked = true;
        auto it = std::max_element(std::begin(sampled_scores), std::end(sampled_scores));  // DELETE LATER.
        double max_score = *it;
        int j = 0;
        for (point2d_t& point : sampled_points) {
            geometry_msgs::Point p;
            p.x = point.x;
            p.y = point.y;
            std_msgs::ColorRGBA color;
            color.r = sampled_scores[j] / max_score;
            color.a = 1.0;
            points.colors.push_back(color);
            points.points.push_back(p);
            j++;
        }
        markers.push_back(points);
        this->rviz_marker_pub_.publish(this->markers_msg_);
        ROS_INFO("presss to continue");
        std::cin.clear();
        std::string x;
        std::cin >> x;
        i++;
    }

    // Store best score and best goal.
    this->exploration_goal_score_ = best_score;
    memcpy(this->exploration_goal_, best_exploration_goal, sizeof(best_exploration_goal));

    ROS_DEBUG("Best exploration goal is Pose(%.3f, %.3f, %.3f) with score %.3f", best_exploration_goal[0], best_exploration_goal[1], best_exploration_goal[2], best_score);

    // Respond with the selected goal.
    res.is_exploration_done = false;
    res.x = best_exploration_goal[0];
    res.y = best_exploration_goal[1];
    res.theta = best_exploration_goal[2];
    return true;
}

void ExploreNode::visualizeFrontiersBestPoseNearby() {
    if (!is_exploration_done_) {
        std::vector<visualization_msgs::Marker>& markers = this->markers_msg_.markers;
        visualization_msgs::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.header.stamp = ros::Time::now();
        arrow.ns = "best_pose_nearby_frontiers";
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.scale.x = 0.3;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.1;
        arrow.color.r = 1.0;  // y arrow.
        arrow.color.g = 1.0;
        arrow.color.a = 0.5;
        arrow.lifetime = ros::Duration();  // forever.
        arrow.frame_locked = true;

        // DELETE LATER
        visualization_msgs::Marker text;
        text.header.frame_id = "map";
        text.header.stamp = ros::Time::now();
        text.ns = "scores_frontiers";
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.scale.z = 0.2;
        text.color.b = 1.0;  //  text.
        text.color.r = 1.0;
        text.color.a = 1.0;
        text.pose.orientation.w = 1.0;
        text.lifetime = ros::Duration();  // forever.
        text.frame_locked = true;

        int id = 0;
        for (const auto& pose : best_goal_per_frontier_) {
            arrow.id = id;
            arrow.pose.position.x = pose[0];
            arrow.pose.position.y = pose[1];
            arrow.pose.orientation.z = sin(pose[2] / 2);
            arrow.pose.orientation.w = cos(pose[2] / 2);
            markers.push_back(arrow);

            // DELETE LATER.
            text.id = id;
            double b, a;
            std::string texto;
            this->gain_evaluator_.computeGain(pose[0], pose[1], b, a, texto);
            text.text = texto;
            text.pose.position.x = pose[0] - 0.5;
            text.pose.position.y = pose[1] - 0.5;
            markers.push_back(text);

            id++;
        }
    }
}

void ExploreNode::visualizeCallback() {
    this->markers_msg_ = visualization_msgs::MarkerArray();
    clearMarkers();
    visualizeFrontiers();
    visualizeActiveArea();
    visualizeFrontiersBestPoseNearby();
    visualizeExplorationGoal();
    this->rviz_marker_pub_.publish(this->markers_msg_);
}

void ExploreNode::visualizeExplorationGoal() {
    if (!is_exploration_done_) {
        std::vector<visualization_msgs::Marker>& markers = this->markers_msg_.markers;
        visualization_msgs::Marker arrow;
        arrow.ns = "exploration_goal";
        arrow.header.frame_id = this->global_frame_;
        arrow.header.stamp = ros::Time::now();
        arrow.pose.position.x = this->exploration_goal_[0];
        arrow.pose.position.y = this->exploration_goal_[1];
        arrow.pose.position.z = 0.0;
        arrow.pose.orientation.z = sin(this->exploration_goal_[2] / 2);  // yaw
        arrow.pose.orientation.w = cos(this->exploration_goal_[2] / 2);
        arrow.scale.x = 0.3;  // arrow length
        arrow.scale.y = 0.1;  // arrow width
        arrow.scale.z = 0.1;  // arrow height
        arrow.color.g = 1.0;  // green.
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration();  // forever.
        markers.push_back(arrow);
    }
}

void ExploreNode::clearMarkers() {
    std::vector<visualization_msgs::Marker>& markers = this->markers_msg_.markers;
    // Create a reset marker that deletes all markers (regardless of namespace).
    visualization_msgs::Marker reset_marker;
    reset_marker.header.frame_id = this->global_frame_;
    reset_marker.header.stamp = ros::Time::now();
    reset_marker.pose.orientation.w = 1.0;
    reset_marker.action = visualization_msgs::Marker::DELETEALL;
    markers.push_back(reset_marker);
}

void ExploreNode::visualizeActiveArea() {
    std::vector<visualization_msgs::Marker>& markers = this->markers_msg_.markers;
    // Common header for all markers.
    std_msgs::Header header;
    header.frame_id = this->global_frame_;
    header.stamp = ros::Time::now();

    // Set up rviz markers for active area
    visualization_msgs::Marker line_strip;
    line_strip.header = header;
    line_strip.ns = "active_area";
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.pose.orientation.w = 1.0;
    line_strip.scale.x = 0.05;  // x is the width of line.
    line_strip.color.r = 1.0;   // red line for active area
    line_strip.color.a = 1.0;
    line_strip.lifetime = ros::Duration();  // forever.
    line_strip.id = 0;
    line_strip.frame_locked = true;

    // Add current active area.
    std::shared_ptr<Map2D> map = this->map_client_.getMap();
    int xmin = map->active_area[0];
    int ymin = map->active_area[1];
    int xmax = map->active_area[2];
    int ymax = map->active_area[3];

    geometry_msgs::Point p;
    p.z = 0.0;
    // convert map coordinates to world coordinates
    map->mapToWorld(xmin, ymin, p.x, p.y);
    line_strip.points.push_back(p);
    map->mapToWorld(xmax, ymin, p.x, p.y);
    line_strip.points.push_back(p);
    map->mapToWorld(xmax, ymax, p.x, p.y);
    line_strip.points.push_back(p);
    map->mapToWorld(xmin, ymax, p.x, p.y);
    line_strip.points.push_back(p);
    map->mapToWorld(xmin, ymin, p.x, p.y);
    line_strip.points.push_back(p);
    markers.push_back(line_strip);
}

void ExploreNode::visualizeFrontiers() {
    // Filter out small size frontiers.
    std::vector<Frontier*> filtered_frontiers;
    int num_filtered_out = 0;
    for (Frontier& frontier : this->frontier_detector_.frontiers) {
        if (frontier.size >= this->min_frontier_size_) {
            filtered_frontiers.emplace_back(&frontier);
            continue;
        }
        num_filtered_out++;
    }

    // ROS_DEBUG("Total number of frontiers: %ld, filtered out %d", this->frontier_detector_.frontiers.size(), num_filtered_out);
    // ROS_DEBUG("Visualizing remaining %lu frontiers", filtered_frontiers.size());
    std::vector<visualization_msgs::Marker>& markers = this->markers_msg_.markers;

    // Common header for all markers.
    std_msgs::Header header;
    header.frame_id = this->global_frame_;
    header.stamp = ros::Time::now();

    // Set up rviz markers for frontier cells.
    visualization_msgs::Marker points;
    points.header = header;
    points.ns = "frontier_cells";
    points.type = visualization_msgs::Marker::POINTS;
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.05;  // x and y scale width/height of point.
    points.scale.y = 0.05;
    points.color.b = 1.0;  // blue points.
    points.color.a = 1.0;
    points.lifetime = ros::Duration();  // forever.
    points.frame_locked = true;

    // Setup rviz marker for frontier centroid
    visualization_msgs::Marker sphere;
    sphere.header = header;
    sphere.ns = "frontier_centroid";
    sphere.type = visualization_msgs::Marker::SPHERE;
    sphere.pose.orientation.w = 1.0;
    sphere.scale.x = 0.3;
    sphere.scale.y = 0.3;
    sphere.scale.z = 0.3;
    sphere.color.b = 1.0;  // Blue color sphere.
    sphere.color.a = 1.0;
    sphere.lifetime = ros::Duration();  // forever.
    sphere.frame_locked = true;

    int id = 0;
    for (const Frontier* frontier : filtered_frontiers) {
        points.id = id;
        sphere.id = id;
        points.points.clear();
        id++;

        // Draw Frontier cells
        for (const auto& frontier_cell : frontier->cells) {
            geometry_msgs::Point p;
            // convert map coordinates to world coordinates
            std::shared_ptr<Map2D> map = this->map_client_.getMap();
            map->mapToWorld(frontier_cell.x, frontier_cell.y, p.x, p.y);
            p.z = 0.0;
            // add to points.
            points.points.push_back(p);
        }

        // Draw Frontier centroid
        sphere.pose.position.x = frontier->centroid[0];
        sphere.pose.position.y = frontier->centroid[1];

        // Add them to marker array
        markers.push_back(points);
        markers.push_back(sphere);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_explore_node");
    ExploreNode explorenode;
    ros::spin();
    return 0;
}
