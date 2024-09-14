#pragma once

#include <memory>
#include <string>

#include "map.h"
#include "map_msgs/OccupancyGridUpdate.h"
#include "nav_msgs/OccupancyGrid.h"
#include "pose_listener.h"
#include "ros/ros.h"

/**
 * It's part of the explore ROS node that initializes a Map2D object based on ros parameters, and handles map update logic.
 * Partially copied from explore_lite package costmap_client with some changes.
 */
class Map2DClient {
   public:
    Map2DClient(ros::NodeHandle& node, PoseListener& pose_listener);
    /**
     * Get the Map2D instance.
     */
    std::shared_ptr<Map2D> getMap();

   private:
    std::shared_ptr<Map2D> map_;
    char cost_translation_table_[256];
    int last_received_map_origin_gx_;  // for efficiency, costmap sends OccupancyGridUpdate message, that is defined w.r.t to the last OccupanGrid message origin.
    int last_received_map_origin_gy_;
    ros::Subscriber occupancy_grid_sub_;
    ros::Subscriber occupancy_grid_update_sub_;

    /**
     *  Update map after receiving nav_msgs/OccupancyGrid message.
     *  This callback function assume the resolution doesn't change, and the received map is completely contained within the map's boundary.
     */
    void updateMap(const nav_msgs::OccupancyGrid::ConstPtr& msg);

    /**
     *  Update map after receiving map_msgs/OccupancyGridUpdate message.
     */
    void updatePartialMap(const map_msgs::OccupancyGridUpdate::ConstPtr& msg);

    /**
     *  Helper function to update active area after each map update (the map update region is indicated by x0,xn,y0,yn).
     *  An active area is a bounding box that indicate the area of the map that has been updated since the last frontier detection.
     */
    void updateActiveArea(int x0, int y0, int xn, int yn);
};