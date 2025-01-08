#include <ros/ros.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#define SLICE_THICK 0.0005
#define SLICE_DIM 0.08
#define SLICE_OFFSET 0.0001

#define BIN_THICK 0.005
#define BIN_W 0.125
#define BIN_H 0.1
#define BIN_L 0.125

void createBin(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double>& pos, std::vector<double>& size, std::string bin_id);
std::pair<std::vector<std::string>, std::vector<std::string>> addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<std::vector<double>> bin_locs);
void pick(moveit::planning_interface::MoveGroupInterface& move_group_interface, const moveit::core::JointModelGroup* joint_model_group, moveit_visual_tools::MoveItVisualTools& visual_tools, const std::vector<double>& pos, const std::string& bin_id, const std::string& slice_id);
void createSlice(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double>& pos, std::vector<double>& size, std::string slice_id);
bool move_to_pos(geometry_msgs::Pose& pose, moveit::planning_interface::MoveGroupInterface& move_group_interface, moveit_visual_tools::MoveItVisualTools& visual_tools, const moveit::core::JointModelGroup* joint_model_group);
void place (moveit::planning_interface::MoveGroupInterface& move_group_interface, const moveit::core::JointModelGroup* joint_model_group, moveit_visual_tools::MoveItVisualTools& visual_tools, const std::vector<double>& pos, const std::string& slice_id);