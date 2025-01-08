#include "pick_and_place.h"


int main(int argc, char** argv) {

    ros::init(argc, argv, "move_group_practice");
    ros::NodeHandle nh;
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
    ros::Duration(2.0).sleep();


    move_group_interface.setMaxVelocityScalingFactor(0.5);
    move_group_interface.setMaxAccelerationScalingFactor(0.5); 
    move_group_interface.setPlanningTime(40.0);
    move_group_interface.allowReplanning(true);

    std::vector<std::vector<double>> bin_locs = {{0.3, 0.25, 0.3}, {0.3, -0.25, 0.3}, {0.1, 0.25, 0.3}, {0.1, -0.25, 0.3}};

    std::pair<std::vector<std::string>, std::vector<std::string>> ids = addCollisionObjects(planning_scene_interface, move_group_interface, bin_locs);
    std::vector<std::string> bin_ids = ids.first;
    std::vector<std::string> slice_ids = ids.second;
    std::vector<double> assem_surf_pos = {0.3, 0, 0.3};

    for (int i = 0; i < bin_locs.size() / 2; i++) {
        pick(move_group_interface, joint_model_group, visual_tools, bin_locs[i], bin_ids[i], slice_ids[i]);
        place(move_group_interface, joint_model_group, visual_tools, {assem_surf_pos[0], assem_surf_pos[1], assem_surf_pos[2] + i * SLICE_THICK}, slice_ids[i]);
    }


    ros::shutdown();
    return 0;

}


bool move_to_pos(geometry_msgs::Pose& pose, moveit::planning_interface::MoveGroupInterface& move_group_interface, 
    moveit_visual_tools::MoveItVisualTools& visual_tools, const moveit::core::JointModelGroup* joint_model_group) {
    
    move_group_interface.setStartStateToCurrentState();  // Ensure the start state is current
    
    move_group_interface.setPoseTarget(pose);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    
    bool success = (move_group_interface.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("moveit_practice", "Visualizing Plan %s", success ? "" : "FAILED");

    if (success) {
        success = (move_group_interface.execute(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        ROS_INFO_NAMED("moveit_practice", "Execution %s", success ? "SUCCESS" : "FAILED");

        if (success) {
            // After execution, update the current state
            move_group_interface.getCurrentState();
        }
    }

    ros::Duration(2).sleep();  // Allow time for the motion to complete
    visual_tools.deleteAllMarkers();
    visual_tools.trigger();
    
    return success;
}



// TODO: Arm is planning but not moving

void pick(moveit::planning_interface::MoveGroupInterface& move_group_interface, const moveit::core::JointModelGroup* joint_model_group, moveit_visual_tools::MoveItVisualTools& visual_tools, 
const std::vector<double>& pos, const std::string& bin_id, const std::string& slice_id) {
    if (pos.size() != 3) {
        throw std::invalid_argument("Positiion vector must be of size 3");
    }

    double z_offset = 0.15;
    
    geometry_msgs::Pose pre_grasp_pose;
    pre_grasp_pose.orientation.x = 1.0;
    pre_grasp_pose.orientation.y = 0.0;
    pre_grasp_pose.orientation.z = 0.0;
    pre_grasp_pose.orientation.w = 0.0; 
    pre_grasp_pose.position.x = pos[0];
    pre_grasp_pose.position.y = pos[1];
    pre_grasp_pose.position.z = pos[2] + z_offset + 0.025;  // 10 cm above the object
    bool success = move_to_pos(pre_grasp_pose, move_group_interface, visual_tools, joint_model_group);



    geometry_msgs::Pose pickup_pose;
    pickup_pose.orientation.x = 1.0;
    pickup_pose.orientation.y = 0.0;
    pickup_pose.orientation.z = 0.0;
    pickup_pose.orientation.w = 0.0; 
    pickup_pose.position.x = pos[0];
    pickup_pose.position.y = pos[1];
    pickup_pose.position.z = pos[2] + z_offset - BIN_H + BIN_THICK + SLICE_THICK + SLICE_OFFSET;
    success = move_to_pos(pickup_pose, move_group_interface, visual_tools, joint_model_group);

    if (success) {
        move_group_interface.attachObject(slice_id, "suction_ee");
        ROS_INFO_NAMED("moveit_practice", "Succesfully picked up slice");
    }


    geometry_msgs::Pose post_grasp_pose;
    post_grasp_pose.orientation.x = 1.0;
    post_grasp_pose.orientation.y = 0.0;
    post_grasp_pose.orientation.z = 0.0;
    post_grasp_pose.orientation.w = 0.0; 
    post_grasp_pose.position.x = pos[0];
    post_grasp_pose.position.y = pos[1];
    post_grasp_pose.position.z = pos[2] + z_offset + 0.025;  
    success = move_to_pos(post_grasp_pose, move_group_interface, visual_tools, joint_model_group);


}

void place (moveit::planning_interface::MoveGroupInterface& move_group_interface, const moveit::core::JointModelGroup* joint_model_group, moveit_visual_tools::MoveItVisualTools& visual_tools, 
const std::vector<double>& pos, const std::string& slice_id) {
    if (pos.size() != 3) {
        throw std::invalid_argument("Positiion vector must be of size 3");
    }

    double z_offset = 0.15;

    double x = pos[0];
    double y = pos[1];
    double z = pos[2];

    geometry_msgs::Pose pre_place_pose;
    pre_place_pose.orientation.x = 1.0;
    pre_place_pose.orientation.y = 0.0;
    pre_place_pose.orientation.z = 0.0;
    pre_place_pose.orientation.w = 0.0;
    pre_place_pose.position.x = pos[0];
    pre_place_pose.position.y = pos[1];
    pre_place_pose.position.z = pos[2] + z_offset + 0.1;

    bool success = move_to_pos(pre_place_pose, move_group_interface, visual_tools, joint_model_group);

    ros::Duration(1).sleep();

    geometry_msgs::Pose place_pose;
    place_pose.orientation.x = 1.0;
    place_pose.orientation.y = 0.0;
    place_pose.orientation.z = 0.0;
    place_pose.orientation.w = 0.0;
    place_pose.position.x = pos[0];
    place_pose.position.y = pos[1];
    place_pose.position.z = pos[2] + z_offset + SLICE_THICK + SLICE_OFFSET;

    success = move_to_pos(place_pose, move_group_interface, visual_tools, joint_model_group);

    if (success) {
        move_group_interface.detachObject(slice_id);
        ROS_INFO_NAMED("moveit_practice", "Succesfully placed up slice");
    }
    ros::Duration(2).sleep();

    geometry_msgs::Pose post_place_pose;
    post_place_pose.orientation.x = 1.0;
    post_place_pose.orientation.y = 0.0;
    post_place_pose.orientation.z = 0.0;
    post_place_pose.orientation.w = 0.0;
    post_place_pose.position.x = pos[0];
    post_place_pose.position.y = pos[1];
    post_place_pose.position.z = pos[2] + z_offset + 0.1;

    success = move_to_pos(pre_place_pose, move_group_interface, visual_tools, joint_model_group);

    ros::Duration(1).sleep();


}


void createSlice(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double>& pos, std::vector<double>& size, std::string slice_id) {
    if (pos.size() != 3 || size.size() != 3) {
        throw std::invalid_argument("Slice Position and Size vectors do not have correct dimensions (3)");
    }

    double x = pos[0];
    double y = pos[1];
    double z = pos[2];

    double w = size[0];
    double l = size[1];
    double thick = size[2];

    moveit_msgs::CollisionObject slice;
    slice.header.frame_id = move_group_interface.getPlanningFrame();
    slice.id = slice_id;


    shape_msgs::SolidPrimitive slice_shape;
    slice_shape.type = shape_msgs::SolidPrimitive::BOX;
    slice_shape.dimensions = {w, l, thick};

    geometry_msgs::Pose slice_pose;
    slice_pose.orientation.w = 1.0;
    slice_pose.position.x = x;
    slice_pose.position.y = y;
    slice_pose.position.z = z + thick / 2;

    slice.primitives.push_back(slice_shape);
    slice.primitive_poses.push_back(slice_pose);
    slice.operation = slice.ADD;
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(slice);

    moveit_msgs::ObjectColor slice_color;
    slice_color.id = slice.id;
    slice_color.color.r = 1.0; 
    slice_color.color.g = 1.0;
    slice_color.color.b = 0.0;
    slice_color.color.a = 0.8;
    std::vector<moveit_msgs::ObjectColor> object_colors = {slice_color};
    planning_scene_interface.applyCollisionObjects(collision_objects, object_colors);
}

void createBin(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<double>& pos, std::vector<double>& size, std::string bin_id) {
    // pos is x, y, z; size is w, l, h
    if (pos.size() != 3 || size.size() != 4) {
        throw std::invalid_argument("Bin Position and Size vectors do not have correct dimensions (3 and 4)");
    }

    double x = pos[0];
    double y = pos[1];
    double z = pos[2];

    double w = size[0];
    double l = size[1];
    double h = size[2];
    double thick = size[3];

   // Create a CollisionObject for the bin
    moveit_msgs::CollisionObject bin;
    bin.header.frame_id = move_group_interface.getPlanningFrame();

    bin.id = bin_id;

    // Create a compound shape for the bin (bottom and 4 walls)
    bin.primitives.resize(5);  // 1 bottom + 4 walls
    bin.primitive_poses.resize(5);

    // Bottom of bin
    shape_msgs::SolidPrimitive bottom;
    bottom.type = shape_msgs::SolidPrimitive::BOX;
    bottom.dimensions = {w, l, thick};
    bin.primitives[0] = bottom;
    bin.primitive_poses[0].position.x = x;
    bin.primitive_poses[0].position.y = y;
    bin.primitive_poses[0].position.z = z - h + thick/2;
    bin.primitive_poses[0].orientation.w = 1;

    // Create the four walls
    // Front
    shape_msgs::SolidPrimitive front_wall;
    front_wall.type = shape_msgs::SolidPrimitive::BOX;
    front_wall.dimensions = {thick, w, h}; 
    bin.primitives[1] = front_wall;
    bin.primitive_poses[1].position.x = x + l / 2 - thick / 2;
    bin.primitive_poses[1].position.y = y;
    bin.primitive_poses[1].position.z = z - h / 2;
    bin.primitive_poses[1].orientation.w = 1;


    // Back
    shape_msgs::SolidPrimitive back_wall;
    back_wall.type = shape_msgs::SolidPrimitive::BOX;
    back_wall.dimensions = {thick, w, h};
    bin.primitives[2] = back_wall;
    bin.primitive_poses[2].position.x = x - l / 2 + thick / 2;
    bin.primitive_poses[2].position.y = y;
    bin.primitive_poses[2].position.z = z - h / 2;
    bin.primitive_poses[2].orientation.w = 1;


    // Left
    shape_msgs::SolidPrimitive left_wall;
    left_wall.type = shape_msgs::SolidPrimitive::BOX;
    left_wall.dimensions = {l, thick, h};
    bin.primitives[3] = left_wall;
    bin.primitive_poses[3].position.x = x;
    bin.primitive_poses[3].position.y = y + w / 2 - thick / 2;
    bin.primitive_poses[3].position.z = z - h / 2;
    bin.primitive_poses[3].orientation.w = 1;


    // Right
    shape_msgs::SolidPrimitive right_wall;
    right_wall.type = shape_msgs::SolidPrimitive::BOX;
    right_wall.dimensions = {l, thick, h};
    bin.primitives[4] = right_wall;
    bin.primitive_poses[4].position.x = x;
    bin.primitive_poses[4].position.y = y - w / 2 + thick / 2;
    bin.primitive_poses[4].position.z = z - h / 2;
    bin.primitive_poses[4].orientation.w = 1;


    bin.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene_interface.applyCollisionObject(bin);
    
}

std::pair<std::vector<std::string>, std::vector<std::string>> addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, 
moveit::planning_interface::MoveGroupInterface& move_group_interface, std::vector<std::vector<double>> bin_locs){
    // add work surface
    moveit_msgs::CollisionObject assem_surf;
    assem_surf.header.frame_id = move_group_interface.getPlanningFrame();

    assem_surf.id = "assembly_area";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = 0.25;
    primitive.dimensions[primitive.BOX_Y] = 0.15;
    primitive.dimensions[primitive.BOX_Z] = 0.05;

    geometry_msgs::Pose assem_pose;
    assem_pose.orientation.w = 1;
    assem_pose.position.x = 0.25;
    assem_pose.position.y = 0.0;
    assem_pose.position.z = 0.3 - 0.025;

    assem_surf.primitives.push_back(primitive);
    assem_surf.primitive_poses.push_back(assem_pose);
    assem_surf.operation = assem_surf.ADD;
    planning_scene_interface.applyCollisionObject(assem_surf);

    // add bins
    std::vector<double> bin_size = {BIN_W, BIN_L, BIN_H, BIN_THICK};
    std::vector<double> slice_size = {SLICE_DIM, SLICE_DIM, SLICE_THICK};

    std::vector<std::string> bin_ids(bin_locs.size());
    std::vector<std::string> slice_ids(bin_locs.size());
    for (int i = 0; i < bin_locs.size(); i++) {
        std::string bin_id = "bin" + std::to_string(i);
        std::string slice_id = "slice" + std::to_string(i);
        bin_ids[i] = bin_id;
        slice_ids[i] = slice_id;
        createBin(planning_scene_interface, move_group_interface, bin_locs[i], bin_size, bin_id);
        std::vector<double> slice_loc = {bin_locs[i][0], bin_locs[i][1], bin_locs[i][2] - bin_size[2] + bin_size[3] + SLICE_OFFSET};
        createSlice(planning_scene_interface, move_group_interface, slice_loc, slice_size, slice_id);
    }


    return std::make_pair(bin_ids, slice_ids);

}