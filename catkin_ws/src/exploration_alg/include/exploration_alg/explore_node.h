#pragma once

#include <visualization_msgs/MarkerArray.h>

#include <list>
#include <string>
#include <vector>

#include "exploration_alg/ExplorationGoalRequest.h"
#include "frontier_detector.h"
#include "gain_evaluator.h"
#include "map.h"
#include "map_client.h"
#include "pose_listener.h"
#include "ros/ros.h"

struct point2d_t {
    double x;
    double y;
};

/**
 * A ROS node that handles exploration. This will run frontier detection and select an exploration goal.
 */
class ExploreNode {
   private:
    ros::NodeHandle node_;
    PoseListener pose_listener_;
    Map2DClient map_client_;
    GainEvaluator gain_evaluator_;
    FrontierDetector frontier_detector_;
    ros::ServiceServer exploration_goal_srv_;
    double exploration_goal_[3];     // Current exploration goal.
    bool is_exploration_done_;       // If exploration is done, then it means there is no more exploration goal.
    double exploration_goal_score_;  // current exploration goal score.
    // Populated by ROS parameters.
    int min_frontier_size_;
    std::string global_frame_;
    // If enabled by visualize_, these will handle rviz visualization.
    bool visualize_;
    ros::Publisher rviz_marker_pub_;
    visualization_msgs::MarkerArray markers_msg_;
    ros::Timer rviz_pub_timer_;
    // If enabled by publish_map_, these will publish maps.
    bool publish_map_;
    ros::Publisher map_pub_;
    ros::Publisher visited_pub_;
    ros::Timer map_pub_timer_;
    // Frontier selection related parameters.
    double sampling_radius_;  // [m].  should be at least greater than sensor max range.
    int number_of_samples_;
    // For visualization.
    std::vector<std::vector<double>> best_goal_per_frontier_;

    bool getExplorationGoal(exploration_alg::ExplorationGoalRequest::Request& req, exploration_alg::ExplorationGoalRequest::Response& res);
    void visualizeFrontiers();
    void visualizeActiveArea();
    void visualizeExplorationGoal();
    void visualizeFrontiersBestPoseNearby();
    void visualizeCallback();
    void clearMarkers();
    void publishMap();

    // Sample free points within a circle centered on (wx,wy) world coordinates.
    // n is desired number of points, but could be less if the # free cells in map < n.
    // In this case, it samples at each grid point once.
    void sampleFreePointsAroundPoint(double wx, double wy, int n, double radius, std::vector<point2d_t>& sampled_free_points);

   public:
    ExploreNode();
};
