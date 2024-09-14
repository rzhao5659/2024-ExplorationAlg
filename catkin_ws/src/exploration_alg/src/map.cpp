#include "map.h"

#include <memory>

#include "ros/ros.h"

Map2D::Map2D(double resolution, double width, double height, double robot_wx, double robot_wy) : resolution(resolution), width(width), height(height) {
    size_x = (int)(width / resolution);
    size_y = (int)(height / resolution);
    // Initialize map data.
    map_data.resize(size_x * size_y);
    for (int i = 0; i < size_x * size_y; i++) {
        map_data[i].occupancy_state = UNKNOWN_SPACE;
        map_data[i].visited_state = false;
    }
    // Compute the origin real world position:
    // The origin (0,0) will be bottom-left corner and its real position will be defined such that the robot current position is at the middle of the grid.
    origin_wx = robot_wx - ((size_x) / 2 + 0.5) * resolution;
    origin_wy = robot_wy - ((size_y) / 2 + 0.5) * resolution;
    ROS_DEBUG("Map's origin real world position is set to (%.3f, %.3f).", origin_wx, origin_wy);
}

void Map2D::mapToWorld(int gx, int gy, double& wx, double& wy) const {
    wx = origin_wx + (gx + 0.5) * resolution;
    wy = origin_wy + (gy + 0.5) * resolution;
}

bool Map2D::worldToMap(double wx, double wy, int& gx, int& gy) const {
    if (wx < origin_wx || wy < origin_wy) return false;

    gx = (int)((wx - origin_wx) / resolution);
    gy = (int)((wy - origin_wy) / resolution);

    if (gx < size_x && gy < size_y) return true;
    return false;
}

int Map2D::getIndex(int gx, int gy) {
    int idx = gy * size_x + gx;
    if (idx >= size_x * size_y) {
        ROS_WARN("Map coordinate (%d, %d) is out of bound, constraining the returned index within map.", gx, gy);
        idx = size_x * size_y - 1;
    }
    return idx;
}
