#ifndef TIAGOSERVER_H
#define TIAGOSERVER_H

// ROS
#include <ros/ros.h>

// CONSTANTS
#include <navigation_automatic_ros/constants/constants.h>

// ACTION CLIENT, ACTION SERVER
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/terminal_state.h>

// MOVE BASE
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

// CUSTOM ACTION
#include <navigation_automatic_ros/MoveDetectAction.h>

// GEOMETRY_MSGS
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

// SENSOR_MSGS
#include <sensor_msgs/LaserScan.h>

// STL
#include <list>

// Action specification for move_base
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/**
 * @author Manuel Barusco
 */
class TiagoServer
{
private:
    ros::NodeHandle nh;                                                               // Node for managing the action server
    actionlib::SimpleActionServer<navigation_automatic_ros::MoveDetectAction> server; // The action server
    MoveBaseClient client;                                                            // Client to the Navigation Stack
    navigation_automatic_ros::MoveDetectFeedback feedback;                            // Feedback message
    navigation_automatic_ros::MoveDetectResult result;                                // Result message
    navigation_automatic_ros::MoveDetectGoal goal;                                    // Goal message

    /**
     * Method for managing all the navigation part of the task
     * @param goal : reference to the MoveDetectAction message goal
     */
    void doNavigation(const navigation_automatic_ros::MoveDetectGoalConstPtr &goal);

public:
    /**
     * Constructor of the class
     * @param name : name of the action server
     */
    TiagoServer(std::string name);

    /**
     * Callback for the main task of the robot
     * @param goal : final pose of the navigation stack
     */
    void navAndDetectCallback(const navigation_automatic_ros::MoveDetectGoalConstPtr &goal);
};

#endif // TIAGOSERVER_H