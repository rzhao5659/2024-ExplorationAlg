
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <cmath>
#include <string>

#include "exploration_alg/ExplorationGoalRequest.h"
#include "ros/ros.h"

class MoveBaseClient {
   private:
    ros::NodeHandle node_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_client_;
    ros::ServiceClient frontier_detection_client_;
    // Ros param
    std::string global_frame_;
    // These are filled from response.
    bool is_exploration_done_;
    double exploration_goal_[3];

   public:
    MoveBaseClient() : move_base_client_("move_base") {
        node_.param<std::string>("global_frame", global_frame_, "map");  // Map frame id

        ROS_INFO("Waiting to connect to move_base server");
        move_base_client_.waitForServer();
        ROS_INFO("Connected to move_base server");

        ROS_INFO("Waiting to connect to explore_node server");
        ros::service::waitForService("/explore/get_goal", ros::Duration(0));
        frontier_detection_client_ = node_.serviceClient<exploration_alg::ExplorationGoalRequest>("/explore/get_goal");
        ROS_INFO("Connected to explore_node server");
    }

    void run() {
        // This line blocks until it receives response from the explore_node server.
        bool server_responded = false;
        while (!server_responded) {
            server_responded = requestExplorationGoal();
        }

        ros::Duration sleep_timer(3.0);

        // Request goal every 3 second.
        while (!is_exploration_done_) {
            // Send exploration goal to move_base
            move_base_msgs::MoveBaseGoal goal;
            geometry_msgs::Point goal_pose;

            goal.target_pose.pose.position.x = exploration_goal_[0];
            goal.target_pose.pose.position.y = exploration_goal_[1];
            goal.target_pose.pose.position.z = 0.0;
            goal.target_pose.pose.orientation.z = sin(exploration_goal_[2] / 2);  // yaw
            goal.target_pose.pose.orientation.w = cos(exploration_goal_[2] / 2);
            goal.target_pose.header.frame_id = global_frame_;
            goal.target_pose.header.stamp = ros::Time::now();
            move_base_client_.sendGoal(goal);

            sleep_timer.sleep();

            // move_base_client_.waitForResult();
            // if (move_base_client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            //     ROS_INFO("Reached exploration goal(%.3f,%.3f,%.3f)", exploration_goal_[0], exploration_goal_[1], exploration_goal_[2] * 180 / M_PI);
            // } else {
            //     // Haven't implemented what to do if it fails. Needs to blacklist an area near it.
            //     ROS_ERROR("Unsuccesful with status: %s", move_base_client_.getState().toString().c_str());
            //     ROS_ERROR("Stopping exploration.");
            //     return;
            // }

            // Request next goal.
            server_responded = false;
            while (!server_responded) {
                server_responded = requestExplorationGoal();
            }
        }
    }

    // Request exploration goal from explore node server. Returns true if the server has responded.
    bool requestExplorationGoal() {
        exploration_alg::ExplorationGoalRequest srv;
        ROS_INFO("Requesting an exploration goal...");
        // Service call will block until it hear response.
        if (frontier_detection_client_.call(srv)) {
            is_exploration_done_ = srv.response.is_exploration_done;
            exploration_goal_[0] = srv.response.x;
            exploration_goal_[1] = srv.response.y;
            exploration_goal_[2] = srv.response.theta;
            if (is_exploration_done_) {
                ROS_INFO("Exploration is finished.");
            } else {
                ROS_INFO("Exploration goal received (%.3f,%.3f,%.3f)", exploration_goal_[0], exploration_goal_[1], exploration_goal_[2] * 180 / M_PI);
            }
            return true;
        }

        ROS_ERROR("Failed to call 'ExplorationGoalRequest' service");
        return false;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_base_client");
    MoveBaseClient move_base_client;
    move_base_client.run();
    ros::spin();
    return 0;
}
