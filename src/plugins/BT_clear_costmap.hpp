#pragma once

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep
#include "rclcpp/rclcpp.hpp"

#include "nav2_msgs/srv/clear_entire_costmap.hpp"

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/basic_types.h"


using namespace std::placeholders;



class ClearCostmap : public BT::SyncActionNode
{
    public:
        ClearCostmap(const std::string& name)
            : BT::SyncActionNode(name, {})
        {
            

            node_ = rclcpp::Node::make_shared("clear_global_costmap_node");
            client_ = node_->create_client<nav2_msgs::srv::ClearEntireCostmap>("/global_costmap/clear_entirely_global_costmap");;
        }



        virtual BT::NodeStatus tick() override
        {
            // Waiting for service /save
            while (!client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service /global_costmap/clear_entirely_global_costmap.");
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "Waiting for service /global_costmap/clear_entirely_global_costmap to appear...");
            }
          

            auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
            auto result_future = client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to call /global_costmap/clear_entirely_global_costmap");
                return BT::NodeStatus::FAILURE;
            }
            else{

                RCLCPP_INFO(node_->get_logger(), "Clear costmap succefully !!!!!");   
                return BT::NodeStatus::SUCCESS;    
                
            }
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_;

    
};