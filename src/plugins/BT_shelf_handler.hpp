#pragma once

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <behaviortree_cpp/action_node.h>


using namespace std::placeholders;


class ShelfHandler : public BT::SyncActionNode
{
    public:
        ShelfHandler(const std::string& name, const BT::NodeConfiguration& config )
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("shelf_handler");
            pub_shelf_up_ = node_->create_publisher<std_msgs::msg::String>("/elevator_up",10);
            pub_shelf_down_ = node_->create_publisher<std_msgs::msg::String>("/elevator_down",10);



        }
        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("shelf_action")};
        }

        virtual BT::NodeStatus tick() override
        {
            std::string shelf_action;

            if (!getInput<std::string>("shelf_action", shelf_action)) {
                // if I can't get this, there is something wrong with your BT.
                // For this reason throw an exception instead of returning FAILURE
                throw BT::RuntimeError("missing required input [shelf_action]");
            }
            RCLCPP_INFO(node_->get_logger(), "Handling shelf action: %s", shelf_action.c_str());

            std_msgs::msg::String action_msg;
            if (shelf_action == "up")
            {   
                pub_shelf_up_->publish(action_msg);
                return BT::NodeStatus::SUCCESS;
            }
            else if (shelf_action == "down")
            {
                pub_shelf_down_->publish(action_msg);
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Unknown action: %s", shelf_action.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_shelf_up_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_shelf_down_;

    };
