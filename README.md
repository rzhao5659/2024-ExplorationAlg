# Project Overview

It's an exploration algorithm implemented from scratch (in `exploration_alg` folder) that combines frontier detection and pose sampling to identify an exploration goal that maximizes score based on information gain and proximity considerations.

This was implemented from scratch for a Mobile Robotic Course. For the grid mapping, I adapted code from explore_lite and costmap2d.

Videos:

- Step-by-step visualization that shows the exploration algorithm computations https://youtu.be/zR9HQgQkwRw
- Comparing difference in behaviors between `explore-lite` (frontier-based ROS package) and this algorithm in a fully autonomous fashion https://www.youtube.com/watch?v=MxkkJAxtDhY

The technical implementation details are in this paper https://arxiv.org/pdf/2404.13767, specifically in Methodology Exploration section.

# Usage

The explore_node continuously runs frontier detection and acts as a server for any ExplorationGoalRequest.
The move base client node is a client both to move base and explore_node: it will request ExplorationGoal from explore_node and send it to move_base. This continues until the exploration is completely done (when explore node sends back a response with exploration_done = true).

## Running the code (completely autonomous)

```bash
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
roslaunch turtlebot3_navigation move_base.launch
roslaunch exploration_alg explore_node.launch
rosrun exploration_alg move_base_client_node
```

## Running the code with step-by-step visualization

This is useful for testing and visualizing how the whole process is done.
This requires rqt to send service request, and use rviz move base interface to send goal.

```bash
roslaunch turtlebot3_gazebo turtlebot3_house.launch
roslaunch turtlebot3_slam turtlebot3_slam.launch
roslaunch turtlebot3_navigation move_base.launch
roslaunch exploration_alg test_explore_node.launch
```
