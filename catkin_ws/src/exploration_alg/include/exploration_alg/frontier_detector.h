#pragma once

#include <list>
#include <memory>
#include <vector>

#include "RTree.h"
#include "cell2d.h"
#include "map.h"
#include "pose_listener.h"
#include "ros/ros.h"

/**
 * Represent a frontier, which is a set of connected frontier cells.
 */
class Frontier {
   public:
    double centroid[2];         // centroid of frontier in world coordinate (wx,wy).
    int size;                   // number of frontier cells that forms this frontier.
    std::list<cell2d_t> cells;  // frontier cells that forms this frontier. Useful for visualization.
    Frontier() {
        centroid[0] = 0.0;
        centroid[1] = 0.0;
        size = 0;
    }

    /**
     * Add a frontier cell into the frontier. The centroid, size and cells are updated.
     */
    void add(cell2d_t& cell) {
        this->cells.emplace_back(cell);
        // Recompute centroid
        this->centroid[0] *= this->size;
        this->centroid[1] *= this->size;
        this->centroid[0] += cell.x;
        this->centroid[1] += cell.y;
        this->size++;
        this->centroid[0] /= this->size;
        this->centroid[1] /= this->size;
    }
};

typedef RTree<cell2d_t*, int, 2, float> RTree_t;
/**
 * Make the interface of RTree easier to use for this application.
 */
class RTreeAdapter {
   private:
    RTree_t tree_;

   public:
    /**
     * Inserts a copy of the given cell to tree_, if the given cell is not duplicated.
     */
    void insert(cell2d_t cell) {
        if (pointSearch(cell) == nullptr) {
            // Create a copy of it in a dynamically allocated memory and store it inside tree_.
            cell2d_t* cell_copy = new cell2d_t(cell);
            int min[2] = {cell_copy->x, cell_copy->y};
            int max[2] = {cell_copy->x, cell_copy->y};
            tree_.Insert(min, max, cell_copy);
        }
    }

    /**
     * Removes a cell from tree_. It will additionally free the memory of this dynamically allocated cell.
     */
    void remove(cell2d_t* cell) {
        // Remove the cell's pointer from tree.
        int min[2] = {cell->x, cell->y};
        int max[2] = {cell->x, cell->y};
        tree_.Remove(min, max, cell);
        // Free memory.
        delete cell;
    }

    /**
     * Return a pointer to the dynamically allocated cell in the tree if it exists.
     */
    cell2d_t* pointSearch(cell2d_t cell) {
        int search_min[2] = {cell.x, cell.y};
        int search_max[2] = {cell.x, cell.y};

        // If it finds the cell, assign it to result.
        cell2d_t* result = nullptr;
        auto searchCallback = [&](cell2d_t* cell) {
            result = cell;
            return false;
        };
        tree_.Search(search_min, search_max, searchCallback);
        return result;
    }

    /**
     * Return a list of cells that lies within the bounding box specified.
     */
    void rangeSearch(int xmin, int ymin, int xmax, int ymax, std::list<cell2d_t*>* cells_within_range) {
        int search_min[2] = {xmin, ymin};
        int search_max[2] = {xmax, ymax};
        // Append to the list each time a cell is found within the requested range.
        auto rangeSearchCallback = [=](cell2d_t* cell) {
            cells_within_range->emplace_back(cell);
            return true;
        };
        tree_.Search(search_min, search_max, rangeSearchCallback);
    }

    /**
     * Return true if the tree is empty.
     */
    bool isEmpty() {
        RTree_t::Iterator it;
        tree_.GetFirst(it);
        if (tree_.IsNull(it)) {
            return true;
        }
        return false;
    }

    /**
     * Returns the tree data structure. Useful for iterating through the tree.
     */
    RTree_t* getTree() { return &tree_; }

    ~RTreeAdapter() {
        // Iterate through every cells in the tree and remove them.
        RTree_t::Iterator it;
        for (tree_.GetFirst(it); !tree_.IsNull(it); tree_.GetNext(it)) {
            cell2d_t* cell = tree_.GetAt(it);
            delete cell;
        }
    }
};

/**
 * Class to detect frontiers with Expanding-Wavefront Frontier Detection. (P Quin, 2021)
 */
class FrontierDetector {
   public:
    std::list<Frontier> frontiers;
    FrontierDetector(ros::NodeHandle& node, std::shared_ptr<Map2D> map, PoseListener* pose_listener);

   private:
    PoseListener* pose_listener_;
    RTreeAdapter frontier_cells_;  // Store frontier cells in a dynamic spatial indexing data structure for quick range query and insertion/removal.
    std::shared_ptr<Map2D> map_;
    ros::Timer execute_timer_;  // will execute periodically runDetection.

    /**
     * Detect new frontier cells and the new frontiers.
     */
    void runDetection();

    /**
     * Perform the expanding-wavefront frontier detection on the active area to update frontier cells on the map.
     */
    void detectNewFrontierCells();

    /**
     * Perform BFS to find all connected components, which are the frontiers.
     * Result is stored in the attribute `frontiers`.
     */
    void getFrontiers();

    /**
     * Get the robot's current position in map coordinates.
     */
    cell2d_t getRobotMapPosition();

    /**
     * Mark a cell as visited in map_.
     */
    void markCellVisited(cell2d_t& cell);

    /**
     * Return true if cell is free.
     */
    bool isCellFree(cell2d_t& cell);

    /**
     * Return true if cell is inside the active area.
     */
    bool isCellInActiveArea(cell2d_t& cell);

    /**
     * Return true if cell is unknown.
     */
    bool isCellUnknown(cell2d_t& cell);

    /**
     * Return true if cell is visited.
     */
    bool isCellVisited(cell2d_t& cell);

    /**
     * Return true if a cell is a frontier cell.
     * A frontier cell is a free cell with at least one adjacent unknown space.
     */
    bool isCellFrontier(cell2d_t& cell);

    /**
     * Return true if a cell was a frontier cell (if it's stored in frontier_cells_).
     */
    bool wasCellFrontier(cell2d_t& cell);

    /**
     * Return the four adjacent cells. This adjacency is used for testing whether a cell is a frontier cell, and detection of new frontier cells.
     */
    void getFourAdjacentCells(cell2d_t& cell, std::list<cell2d_t>& adj_cells);

    /**
     * Return the eight adjacent cells. This adjacency is used to test connectivity of frontier cells for formation of frontiers.
     */
    void getEightAdjacentCells(cell2d_t& cell, std::list<cell2d_t>& adj_cells);
};