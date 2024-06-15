#pragma once

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep
#include "rclcpp/rclcpp.hpp"

#include "rb1_shelf_msgs/srv/find_shelf.hpp"


#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <behaviortree_cpp/action_node.h>
#include "behaviortree_cpp/basic_types.h"


using namespace std::placeholders;



class FindShelfClient : public BT::SyncActionNode
{
    public:
        FindShelfClient(const std::string& name, const BT::NodeConfiguration& config)
            : BT::SyncActionNode(name, config)
        {
            

            node_ = rclcpp::Node::make_shared("find_object_client");
            client_ = node_->create_client<rb1_shelf_msgs::srv::FindShelf>("/find_shelf_server");;
        }

        static BT::PortsList providedPorts()
        {
            return{ 
                BT::OutputPort<geometry_msgs::msg::Point>("position_object_found"),
                BT::InputPort<std::string>("object_find"),
            
            };
            
        }


        virtual BT::NodeStatus tick() override
        {
            if (!getInput<std::string>("object_find", object_find_)) {
                // if I can't get this, there is something wrong with your BT.
                // For this reason throw an exception instead of returning FAILURE
                throw BT::RuntimeError("missing required input [object_find]");
            }
            // Waiting for service /save
            while (!client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service /find_shelf_server.");
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "Waiting for service /find_shelf_server to appear...");
            }
          

            auto request = std::make_shared<rb1_shelf_msgs::srv::FindShelf::Request>();
            request->object_find =object_find_ ;
            auto result_future = client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to call /find_shelf_server");
                return BT::NodeStatus::FAILURE;
            }
            else{

                auto result = result_future.get();
                if(result->success !=true){

                    RCLCPP_ERROR(node_->get_logger(), "No se encontro [%s]",request->object_find.c_str());
                    return BT::NodeStatus::FAILURE;
                }
                else{
                    RCLCPP_DEBUG(node_->get_logger(), "Se encontro [%s]",request->object_find.c_str());
                    geometry_msgs::msg::Point pose_object = result->shelf_position;

          
                    setOutput<geometry_msgs::msg::Point>("position_object_found",pose_object);
                    //setOutput("position_shelf_found",pose_shelf);

                   RCLCPP_DEBUG(node_->get_logger(), "Posición del [%s]  r [%.3f]  theta [%.3f]",request->object_find.c_str(),pose_object.x,pose_object.y);
                   return BT::NodeStatus::SUCCESS;    
                }
                
            }
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<rb1_shelf_msgs::srv::FindShelf>::SharedPtr client_;
        std::string object_find_;

    
};