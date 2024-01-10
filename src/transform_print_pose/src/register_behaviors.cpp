#include <behaviortree_cpp/bt_factory.h>
#include <pluginlib/class_list_macros.hpp>
#include <transform_print_pose/transform_print_pose.hpp>

#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

namespace transform_print_pose
{
class TransformPrintPoseBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(
      BT::BehaviorTreeFactory& factory,
      [[maybe_unused]] const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<TransformPrintPose>(factory, "TransformPrintPose");
  }
};
}  // namespace transform_print_pose

PLUGINLIB_EXPORT_CLASS(transform_print_pose::TransformPrintPoseBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
