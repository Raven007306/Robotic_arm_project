#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <gazebo_msgs/srv/spawn_entity.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

void spawn_box_in_gazebo(const rclcpp::Node::SharedPtr &node)
{
    auto client = node->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
    while (!client->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_INFO(node->get_logger(), "Waiting for /spawn_entity service...");
    }

    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    request->name = "box";
    request->xml = R"(
        <sdf version="1.6">
          <model name="box">
            <static>true</static>
            <link name="link">
              <visual name="visual">
                <geometry>
                  <box>
                    <size>1.5 1.5 1.5</size>
                  </box>
                </geometry>
              </visual>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>1.5 1.5 1.5</size>
                  </box>
                </geometry>
              </collision>
            </link>
          </model>
        </sdf>
    )";
    request->initial_pose.position.x = 2.0;
    request->initial_pose.position.y = 0.0;
    request->initial_pose.position.z = 0.75;

    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Box spawned in Gazebo.");
    }
    else
    {
        RCLCPP_ERROR(node->get_logger(), "Failed to spawn box in Gazebo.");
    }
}

void add_box_to_moveit(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.id = "box";
    collision_object.header.frame_id = "world";

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {1.5, 1.5, 1.5};

    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 2.0;
    box_pose.position.y = 0.0;
    box_pose.position.z = 0.75;

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    planning_scene_interface.applyCollisionObject(collision_object);
}


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("kuka_collision_avoidance_demo");

    RCLCPP_INFO(node->get_logger(), "Starting collision avoidance demo...");

    // Spawn box in Gazebo
    spawn_box_in_gazebo(node);

    // Move group interface for controlling the robot arm
    moveit::planning_interface::MoveGroupInterface move_group(node, "kuka_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Visualization tools
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "world", "/rviz_visual_tools");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    visual_tools.trigger();

    // Add box to the planning scene
    add_box_to_moveit(planning_scene_interface);

    // Set planning time
    move_group.setPlanningTime(10.0);

    // Set a target pose for the robot arm
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 1.5;
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
        RCLCPP_ERROR(node->get_logger(), "Motion plan failed.");
    }

    // Visualize the target pose in RViz
    visual_tools.publishAxisLabeled(target_pose, "Target Pose");
    visual_tools.publishText(
        Eigen::Isometry3d::Identity(), "Motion Executed", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    rclcpp::shutdown();
    return 0;
}
