#pragma once

#include <behaviortree_cpp/decorator_node.h>

class ForceSuccess : public BT::DecoratorNode {
public:
  ForceSuccess(const std::string &name, const BT::NodeConfiguration &config)
      : BT::DecoratorNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

protected:
  BT::NodeStatus tick() override {
    BT::NodeStatus child_status = child_node_->executeTick();
    return BT::NodeStatus::SUCCESS; // Fuerza el éxito siempre
  }
};
