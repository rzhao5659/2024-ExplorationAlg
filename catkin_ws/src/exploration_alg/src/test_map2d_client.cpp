#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "map_client.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"

// This code will test Map2DClient, MAP2D, pose listener.
void publishMapOccupancy(ros::Publisher& map_pub, std::shared_ptr<Map2D>& map) {
    nav_msgs::OccupancyGrid occupancy_grid_msg;
    occupancy_grid_msg.header.stamp = ros::Time::now();
    occupancy_grid_msg.header.frame_id = "map";
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
    occupancy_grid_msg.data.resize(occupancy_grid_msg.info.width * occupancy_grid_msg.info.height, 0);  // initialize all cells with 0 (unoccupied)
    for (int i = 0; i < map->size_x * map->size_y; i++) {
        occupancy_grid_msg.data[i] = map->map_data[i].occupancy_state;
    }
    map_pub.publish(occupancy_grid_msg);
}

void publishMapVisited(ros::Publisher& map_pub, std::shared_ptr<Map2D>& map, PoseListener& pose_listener) {
    // Normally visited layer is modified only by frontier detector.
    // I am modifying it so that it marks 1 to all robot trajectory to test map coordinate transformations to test.
    nav_msgs::OccupancyGrid occupancy_grid_msg;
    occupancy_grid_msg.header.stamp = ros::Time::now();
    occupancy_grid_msg.header.frame_id = "map";
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
    occupancy_grid_msg.data.resize(occupancy_grid_msg.info.width * occupancy_grid_msg.info.height, 0);

    // Get robot current pose
    double wx, wy, wtheta;
    pose_listener.getRobotWorldPosition(wx, wy, wtheta);
    ROS_DEBUG("Robot's position is currently (%.3f, %.3f, %.3f).", wx, wy, wtheta * 180 / M_PI);
    // Mark the robot position visited in the grid.
    int gx, gy;
    map->worldToMap(wx, wy, gx, gy);
    int idx = map->getIndex(gx, gy);
    map->map_data[idx].visited_state = true;

    // Update grid msg according to visited state.
    for (int i = 0; i < map->size_x * map->size_y; i++) {
        occupancy_grid_msg.data[i] = map->map_data[i].visited_state == true ? 100 : 0;
    }
    map_pub.publish(occupancy_grid_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_map_node");
    ros::NodeHandle node;
    PoseListener pose_listener(&node);
    Map2DClient map_client(node, pose_listener);

    // Map publisher
    std::shared_ptr<Map2D> map = map_client.getMap();
    ros::Publisher map_pub = node.advertise<nav_msgs::OccupancyGrid>("/test/map/occupied", 10);
    ros::Publisher visited_pub = node.advertise<nav_msgs::OccupancyGrid>("/test/map/visited", 10);

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        ros::spinOnce();
        publishMapOccupancy(map_pub, map);
        publishMapVisited(visited_pub, map, pose_listener);
        loop_rate.sleep();
    }
    return 0;
}
