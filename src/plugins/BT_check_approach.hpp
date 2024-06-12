
#pragma once

#include <functional>
#include <limits>
#include <chrono>
#include <unistd.h> // Used by sleep
#include <thread> // Used for std::this_thread::sleep_for

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <behaviortree_cpp/action_node.h>



using namespace std::placeholders;


class CheckApproach : public BT::StatefulActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    CheckApproach(const std::string& name , const BT::NodeConfiguration& config)
      : StatefulActionNode(name, config)
    {

    }
      static BT::PortsList providedPorts()
      {
        return{ 
          BT::InputPort<std::string>("control_state_input")
        };
      }


    // this function is invoked once at the beginning.
    BT::NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    BT::NodeStatus onRunning() override;

    // callback to execute if the action was aborted by another node
    void onHalted() override;

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;

    std::chrono::system_clock::time_point _completion_time;

};

//-------------------------


BT::NodeStatus CheckApproach::onStart()
{
  // We use this counter to simulate an action that takes a certain
  // amount of time to be completed (200 ms)

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus CheckApproach::onRunning()
{
  // Pretend that we are checking if the reply has been received
  // you don't want to block inside this function too much time.
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  std::string control_state;

  if (!getInput<std::string>("control_state_input", control_state)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [control_state]");
  }

  if(control_state == "END")
  {
    std::cout << "[ MoveBase: FINISHED ]" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void CheckApproach::onHalted()
{
  printf("[ MoveBase: ABORTED ]");
}