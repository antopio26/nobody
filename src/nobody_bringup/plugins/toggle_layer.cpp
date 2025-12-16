#include <string>
#include <memory>
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nobody_bringup
{

class ToggleLayer : public nav2_behavior_tree::BtServiceNode<rcl_interfaces::srv::SetParameters>
{
public:
  ToggleLayer(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf)
  : BtServiceNode<rcl_interfaces::srv::SetParameters>(xml_tag_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<std::string>("layer_name", "static_layer", "Name of the layer param to toggle"),
      BT::InputPort<bool>("enabled", true, "Enable or disable the layer")
    });
  }

  void on_tick() override
  {
    std::string layer_name;
    bool enabled;
    getInput("layer_name", layer_name);
    getInput("enabled", enabled);

    request_->parameters.clear();
    rcl_interfaces::msg::Parameter param;
    param.name = layer_name + ".enabled";
    param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    param.value.bool_value = enabled;
    request_->parameters.push_back(param);
  }
};

} // namespace nobody_bringup

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nobody_bringup::ToggleLayer>("ToggleLayer");
}
