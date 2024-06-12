#pragma once

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep
#include "rclcpp/rclcpp.hpp"

#include "rb1_shelf_msgs/srv/approach_shelf.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/basic_types.h"


using namespace std::placeholders;



class ApproachShelfClient : public BT::SyncActionNode
{
    public:
        ApproachShelfClient(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            

            node_ = rclcpp::Node::make_shared("approach_shelf_client");
            client_ = node_->create_client<rb1_shelf_msgs::srv::ApproachShelf>("/approach_shelf_server");
        }

        static BT::PortsList providedPorts()
        {
            return{ 
                BT::OutputPort<std::string>("control_state_output"),
                BT::InputPort<std::string>("type_control"),
            };
        }


        virtual BT::NodeStatus tick() override
        {
            // Pido el tipo de control efectuado
            if (!getInput<std::string>("type_control", type_control_)) {
                // if I can't get this, there is something wrong with your BT.
                // For this reason throw an exception instead of returning FAILURE
                throw BT::RuntimeError("missing required input [type_control]");
            }

            // Waiting for service /save
            while (!client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service /approach_shelf_server.");
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "Waiting for service /approach_shelf_server to appear...");
            }
          

            auto request = std::make_shared<rb1_shelf_msgs::srv::ApproachShelf::Request>();
            request->type_control = type_control_;

            auto result_future = client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to call /approach_shelf_server");
                return BT::NodeStatus::FAILURE;
            }
            else{
                auto result = result_future.get();
                if(result->success !=true){

                    RCLCPP_ERROR(node_->get_logger(), "------------");
                    return BT::NodeStatus::FAILURE;
                }
                else{
                    RCLCPP_INFO(node_->get_logger(), "Enviando comandos de velocidad !!!!");
                    // Envio el estatus del control
                    std::string control_status = result->state_control_output;
                    setOutput<std::string>("control_state_output",control_status);
                    RCLCPP_INFO(node_->get_logger(), "control_status [%s]",control_status.c_str());
                    return BT::NodeStatus::SUCCESS;    
                }
                
            }
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<rb1_shelf_msgs::srv::ApproachShelf>::SharedPtr client_;
        std::string control_status;
        std::string type_control_;


    
};