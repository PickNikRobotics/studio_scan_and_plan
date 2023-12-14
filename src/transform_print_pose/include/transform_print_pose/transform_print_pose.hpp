#pragma once

#include <behaviortree_cpp/action_node.h>



namespace transform_print_pose
{
/**
 * @brief TODO(...)
 */
class TransformPrintPose : public BT::SyncActionNode
{
public:
  /**
   * @brief Constructor for the transform_print_pose behavior.
   * @param name The name of a particular instance of this Behavior. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @param config This contains runtime configuration info for this Behavior, such as the mapping between the Behavior's data ports on the behavior tree's blackboard. This will be set by the behavior tree factory when this Behavior is created within a new behavior tree.
   * @details An important limitation is that the members of the base Behavior class are not instantiated until after the initialize() function is called, so these classes should not be used within the constructor.
   */
  TransformPrintPose(const std::string& name, const BT::NodeConfiguration& config);

  /**
   * @brief Implementation of the required providedPorts() function for the transform_print_pose Behavior.
   * @details The BehaviorTree.CPP library requires that Behaviors must implement a static function named providedPorts() which defines their input and output ports. If the Behavior does not use any ports, this function must return an empty BT::PortsList.
   * This function returns a list of ports with their names and port info, which is used internally by the behavior tree.
   * @return transform_print_pose does not use expose any ports, so this function returns an empty list.
   */
  static BT::PortsList providedPorts();

  /**
   * @brief Implementation of BT::SyncActionNode::tick() for TransformPrintPose.
   * @details This function is where the Behavior performs its work when the behavior tree is being run. Since TransformPrintPose is derived from BT::SyncActionNode, it is very important that its tick() function always finishes very quickly. If tick() blocks before returning, it will block execution of the entire behavior tree, which may have undesirable consequences for other Behaviors that require a fast update rate to work correctly.
   */
  BT::NodeStatus tick() override;
  
};
}  // namespace transform_print_pose
