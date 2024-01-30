#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.141592

using namespace std::chrono_literals; // For using milliseconds

int main(int argc, char * argv[])
{

  // Create a ROS logger
  auto const LOGGER = rclcpp::get_logger("ur_camera_position");

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("ur_camera_position", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);

  //const moveit::core::JointModelGroup *joint_model_group_arm =
  //    move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);

  static const std::string CAMERA_LINK = "camera_link";

  // Get position and quaternion of camera
  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped current_pose = move_group_arm.getCurrentPose(CAMERA_LINK);
    RCLCPP_INFO(move_group_node->get_logger(), "Camera Position: (%.6f, %.6f, %.6f)",
                current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
    RCLCPP_INFO(move_group_node->get_logger(), "Camera Quaternion : (%.6f, %.6f, %.6f, %.6f)",
                current_pose.pose.orientation.x, current_pose.pose.orientation.y,
                current_pose.pose.orientation.z, current_pose.pose.orientation.w);
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
