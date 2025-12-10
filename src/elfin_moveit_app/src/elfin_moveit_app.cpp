#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "elfin_moveit_app",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("elfin_moveit_app");

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "elfin_arm");

  double x = node->get_parameter_or("x", -0.3);
  double y = node->get_parameter_or("y",  0.0);
  double z = node->get_parameter_or("z",  0.7);

  double roll_deg  = node->get_parameter_or("roll_deg",  0.0);
  double pitch_deg = node->get_parameter_or("pitch_deg", 0.0);
  double yaw_deg   = node->get_parameter_or("yaw_deg",   0.0);

  geometry_msgs::msg::Pose target_pose;

  const double roll  = roll_deg  * M_PI / 180.0;
  const double pitch = pitch_deg * M_PI / 180.0;
  const double yaw   = yaw_deg   * M_PI / 180.0;

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  target_pose.orientation = tf2::toMsg(q);

  target_pose.position.x = x;
  target_pose.position.y = y;
  target_pose.position.z = z;

  move_group_interface.setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(move_group_interface.plan(plan));

  if (success) {
    move_group_interface.execute(plan);
    RCLCPP_INFO(logger, "Trajectory executed successfully!");
  } else {
    RCLCPP_ERROR(logger, "Planning failed!");
  }

  rclcpp::shutdown();
  return 0;
}
