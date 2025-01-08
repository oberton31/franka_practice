#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <franka_gripper/MoveActionGoal.h>
#include <franka_gripper/GraspActionGoal.h>


void moveToPose(ros::Publisher& pose_pub, const std::vector<double>& pose) {
    geometry_msgs::PoseStamped pose_cmd;
    pose_cmd.header.stamp = ros::Time::now();
    pose_cmd.header.frame_id = "panda_link0";
    
    pose_cmd.pose.position.x = pose[0];
    pose_cmd.pose.position.y = pose[1];
    pose_cmd.pose.position.z = pose[2];
    
    pose_cmd.pose.orientation.x = pose[3];
    pose_cmd.pose.orientation.y = pose[4];
    pose_cmd.pose.orientation.z = pose[5];
    pose_cmd.pose.orientation.w = pose[6];

    pose_pub.publish(pose_cmd);
    ros::Duration(3.0).sleep();  

}

void openGripper(ros::Publisher& move_pub) {
    franka_gripper::MoveActionGoal move_goal;
    move_goal.goal.width = 0.08;  
    move_goal.goal.speed = 0.1;

    move_pub.publish(move_goal);
    ros::Duration(3.0).sleep();  

}

void graspObj(ros::Publisher& grasp_pub) {
    franka_gripper::GraspActionGoal grasp_goal;

    grasp_goal.goal.width = 0.03;
    grasp_goal.goal.epsilon.inner = 0.005;
    grasp_goal.goal.epsilon.outer = 0.005;
    grasp_goal.goal.speed = 0.1;
    grasp_goal.goal.force = 5.0;
    grasp_pub.publish(grasp_goal);
    ros::Duration(3.0).sleep();  
}

void pickUpObj(ros::Publisher& pose_pub, ros::Publisher& move_pub, ros::Publisher& grasp_pub, const std::vector<double>& grasp_pose) {
    std::vector<double> pre_post_grasp_pose = grasp_pose;
    pre_post_grasp_pose[2] += 0.1;

    openGripper(move_pub);
    moveToPose(pose_pub, pre_post_grasp_pose);
    moveToPose(pose_pub, grasp_pose);
    graspObj(grasp_pub);
    moveToPose(pose_pub, pre_post_grasp_pose);
}

void placeObj(ros::Publisher& pose_pub, ros::Publisher& move_pub, const std::vector<double>& place_pose) {
    std::vector<double> pre_post_place_pose = place_pose;
    pre_post_place_pose[2] += 0.1;

    moveToPose(pose_pub, pre_post_place_pose);
    moveToPose(pose_pub, place_pose);
    openGripper(move_pub);
    moveToPose(pose_pub, pre_post_place_pose);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_commander");
    ros::NodeHandle nh;
    
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_example_controller/equilibrium_pose", 10);
    ros::Publisher move_pub = nh.advertise<franka_gripper::MoveActionGoal>("/franka_gripper/move/goal", 10);
    ros::Publisher grasp_pub = nh.advertise<franka_gripper::GraspActionGoal>("/franka_gripper/grasp/goal", 10);
    ros::Rate rate(1);

    std::vector<double> grasp_pose = {0.503, -0.2327, 0.49, 1, 0, 0, 0};
    std::vector<double> place_pose = {0.503, 0.2327, 0.5, 1, 0, 0, 0};

    while (pose_pub.getNumSubscribers() == 0) {
        if (!ros::ok()) return 1;
        rate.sleep();
    }

    pickUpObj(pose_pub, move_pub, grasp_pub, grasp_pose);
    placeObj(pose_pub, move_pub, place_pose);
    ROS_INFO("Finished executing commands");
    ros::spin();
    return 0;
}
