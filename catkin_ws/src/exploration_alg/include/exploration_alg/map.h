#pragma once

#include <memory>
#include <vector>

const char UNKNOWN_SPACE = -1;
const char FREE_SPACE = 0;
const char OCCUPIED_SPACE = 100;

/**
 * Represents the map's data at each grid position.
 */
struct map_node_t {
    bool visited_state;
    char occupancy_state;
};

/**
 * A class that represents a 2D static grid map.  Its origin (0,0) is at the bottom-left corner and its real world position is represented by origin_wx and origin_wy.
 * The map size in meters is (width, height) and this doesn't change dynamically.
 */
class Map2D {
   public:
    std::vector<map_node_t> map_data;  // Map data.
    int size_x;                        // [#cells]
    int size_y;                        // [#cells]
    double resolution;
    double width;
    double height;
    double origin_wx;  // origin is the bottom-left corner of the grid (0,0).  wx means x in world coordinate.
    double origin_wy;

    // A box (xmin,ymin,xmax,ymax) that expresses the area where occupancy state has changed.
    // This will keep becoming bigger as more occupancy grid data is received.
    // When frontier detection runs, it will process this area and reset this value to robot position.
    int active_area[4];

   public:
    Map2D(double resolution, double width, double height, double robot_wx, double robot_wy);
    /**
     * Map given (gx, gy) map coordintes to (wx, wy) world coordinates.
     */
    void mapToWorld(int gx, int gy, double& wx, double& wy) const;

    /**
     * Map given (wx, wy) world coordinates to (gx, gy) map coordintes.
     * Returns true if succesfull. May fail due to out of map boundary.
     */
    bool worldToMap(double wx, double wy, int& gx, int& gy) const;

    /**
     * Return the corresponding index in the 1D array map for the 2D grid coordinates (gx, gy).
     */
    int getIndex(int gx, int gy);
};