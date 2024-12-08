#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("kuka_collision_avoidance_demo");

    RCLCPP_INFO(node->get_logger(), "Starting collision avoidance demo...");

    // Move group interface for controlling the robot arm
    moveit::planning_interface::MoveGroupInterface move_group(node, "kuka_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Visualization tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "base_link", "/rviz_visual_tools");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    // Define a collision object in the planning scene
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    collision_object.id = "box";

    // Define the shape and pose of the collision object
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {1.5, 1.5, 1.5}; // Dimensions: x, y, z

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 2.0;  // Moved to avoid collision with the robot
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.2;
    box_pose.orientation.w = 1.0;

    collision_object.primitives.push_back(box);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    // Add the collision object to the planning scene
    planning_scene_interface.applyCollisionObject(collision_object);
    RCLCPP_INFO(node->get_logger(), "Added collision object to planning scene");

    // Visualize the collision object in RViz
    visual_tools.publishText(
        Eigen::Isometry3d::Identity(), "Collision Object Added", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Set planning time
    move_group.setPlanningTime(10.0);

    // Set a target pose for the robot arm
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 1.5;  // Within robot's reach
    target_pose.position.y = 1.0;
    target_pose.position.z = 0.25;
    target_pose.orientation.w = 1.0;

    move_group.setPoseTarget(target_pose);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(node->get_logger(), "Motion plan successful! Executing...");
        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Motion plan failed");
    }

    // Visualize the target pose in RViz
    visual_tools.publishAxisLabeled(target_pose, "Target Pose");
    visual_tools.publishText(
        Eigen::Isometry3d::Identity(), "Motion Executed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}
