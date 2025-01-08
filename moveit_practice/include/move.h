#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <ros/ros.h>
#include <shape_msgs/SolidPrimitive.h>  // Add this header for SolidPrimitive


void move_to_pos(geometry_msgs::Pose& pose, moveit::planning_interface::MoveGroupInterface& move_group_interface, 
    moveit_visual_tools::MoveItVisualTools& visual_tools, const moveit::core::JointModelGroup* joint_model_group);