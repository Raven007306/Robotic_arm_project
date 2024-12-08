#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class PickAndPlace : public rclcpp::Node, public std::enable_shared_from_this<PickAndPlace>
{
public:
    PickAndPlace() : Node("pick_and_place_node")
    {
        // Initialize MoveIt interfaces
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            std::enable_shared_from_this<PickAndPlace>::shared_from_this(), "kuka_arm");
        planning_scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

        // Add a collision object and perform pick and place
        addCollisionObject();
        performPickAndPlace();
    }

private:
    void addCollisionObject()
    {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = move_group_->getPlanningFrame();
        collision_object.id = "target_object";

        // Define the object's dimensions and pose
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {0.1, 0.1, 0.1}; // Box dimensions

        geometry_msgs::msg::Pose box_pose;
        box_pose.position.x = 0.5;
        box_pose.position.y = 0.0;
        box_pose.position.z = 0.5;
        box_pose.orientation.w = 1.0;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        planning_scene_interface_->applyCollisionObject(collision_object);
        RCLCPP_INFO(this->get_logger(), "Collision object added to the planning scene.");
    }

    void performPickAndPlace()
    {
        // Set target pose for picking
        geometry_msgs::msg::PoseStamped pick_pose;
        pick_pose.header.frame_id = move_group_->getPlanningFrame();
        pick_pose.pose.position.x = 0.5;
        pick_pose.pose.position.y = 0.0;
        pick_pose.pose.position.z = 0.5;
        pick_pose.pose.orientation.w = 1.0;

        move_group_->setPoseTarget(pick_pose);
        if (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            move_group_->move();
            RCLCPP_INFO(this->get_logger(), "Pick motion executed.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan pick motion.");
            return;
        }

        // Set target pose for placing
        geometry_msgs::msg::PoseStamped place_pose;
        place_pose.header.frame_id = move_group_->getPlanningFrame();
        place_pose.pose.position.x = 0.5;
        place_pose.pose.position.y = 0.5;
        place_pose.pose.position.z = 0.5;
        place_pose.pose.orientation.w = 1.0;

        move_group_->setPoseTarget(place_pose);
        if (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            move_group_->move();
            RCLCPP_INFO(this->get_logger(), "Place motion executed.");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan place motion.");
        }
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickAndPlace>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
