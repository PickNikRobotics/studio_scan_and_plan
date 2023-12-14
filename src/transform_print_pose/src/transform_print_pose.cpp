#include <iostream>

#include <rclcpp/logging.hpp>
#include <transform_print_pose/transform_print_pose.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <moveit_studio_behavior_interface/check_for_error.hpp>

namespace
{
constexpr auto kPortIDInputPose = "input_pose";
const rclcpp::Logger kLogger = rclcpp::get_logger("TransformPrintPose");
}  // namespace

namespace transform_print_pose
{
TransformPrintPose::TransformPrintPose(const std::string& name, const BT::NodeConfiguration& config)
  : BT::SyncActionNode(name, config)
{
}

BT::PortsList TransformPrintPose::providedPorts()
{
  // TODO(...)
  return BT::PortsList({
      BT::InputPort<geometry_msgs::msg::PoseStamped>(kPortIDInputPose),
  });
}

BT::NodeStatus TransformPrintPose::tick()
{
  // TODO(...)
  // Return SUCCESS once the work has been completed.
  const auto maybe_input_pose = getInput<geometry_msgs::msg::PoseStamped>(kPortIDInputPose);

  if (const auto error = moveit_studio::behaviors::maybe_error(maybe_input_pose); error)
  {
    RCLCPP_ERROR(kLogger, "Failed to get good pose values");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_ERROR(kLogger, "Transform is: %s", maybe_input_pose.value().header.frame_id.c_str());

  return BT::NodeStatus::SUCCESS;
}

}  // namespace transform_print_pose
