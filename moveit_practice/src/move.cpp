#include "move.h"
int main(int argc, char** argv) {

    ros::init(argc, argv, "move_group_practice");
    ros::NodeHandle nh;


    // ROS spinning must be running for the MoveGroupInterface to get information
    // about the robot's state. One way to do this is to start an AsyncSpinner
    // beforehand.
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup* joint_model_group =
    move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);


    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();

    geometry_msgs::Pose waypoint1;
    waypoint1.orientation.w = cos(M_PI / 4);  // w = cos(90°/2)
    waypoint1.orientation.x = sin(M_PI / 4);  // x = sin(90°/2)
    waypoint1.orientation.y = 0;
    waypoint1.orientation.z = 0;
    waypoint1.position.x = 0;
    waypoint1.position.y = 0.5;
    waypoint1.position.z = 0.4;

    geometry_msgs::Pose waypoint2;
    waypoint2.orientation.x = 1.0;
    waypoint2.position.x = 0.5;
    waypoint2.position.y = 0;
    waypoint2.position.z = 0.1;

    // geometry_msgs::Pose waypoint3;
    // waypoint3.orientation.x = 1.0;
    // waypoint3.position.x = -0.2;
    // waypoint3.position.y = 0.5;
    // waypoint3.position.z = 0.5;

    // std::vector<geometry_msgs::Pose> waypoints {waypoint1, waypoint2, waypoint3};

    moveit_msgs::CollisionObject box;
    box.header.frame_id = move_group_interface.getPlanningFrame();

    box.id = "box1";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 1.5;
    primitive.dimensions[primitive.BOX_Y] = 0.1;
    primitive.dimensions[primitive.BOX_Z] = 0.5;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x = 0;
    box_pose.position.y = 0.3;
    box_pose.position.z = 0.25;

    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(box);

    planning_scene_interface.addCollisionObjects(collision_objects);
    move_group_interface.setMaxVelocityScalingFactor(0.3);
    move_group_interface.setMaxAccelerationScalingFactor(0.3); 
    move_group_interface.setPlanningTime(10.0);

    move_to_pos(waypoint1, move_group_interface, visual_tools, joint_model_group);
    ros::Duration(2).sleep();
    move_to_pos(waypoint2, move_group_interface, visual_tools, joint_model_group);
    ros::Duration(2).sleep();
    ros::shutdown();
    return 0;
}

void move_to_pos(geometry_msgs::Pose& pose, moveit::planning_interface::MoveGroupInterface& move_group_interface, 
    moveit_visual_tools::MoveItVisualTools& visual_tools, const moveit::core::JointModelGroup* joint_model_group)  {
    move_group_interface.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("moveit_practice", "Visualizing Plan %s", success ? "" : "FAILED");
    visual_tools.publishTrajectoryLine(plan.trajectory_, joint_model_group);
    visual_tools.trigger();
    ros::Duration(4).sleep();

    visual_tools.deleteAllMarkers();
    visual_tools.trigger(); 

    if (success) {
        move_group_interface.execute(plan);
    }
}
