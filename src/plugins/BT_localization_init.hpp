#pragma once

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep
#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/empty.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <behaviortree_cpp/action_node.h>

using namespace std::placeholders;

class LocalizationInit : public BT::SyncActionNode
{
    public:
        LocalizationInit(const std::string& name)
            : BT::SyncActionNode(name, {})
        {
            

            node_ = rclcpp::Node::make_shared("localization_init_node");
            client_ = node_->create_client<std_srvs::srv::Empty>("/reinitialize_global_localization");
            pub_cmd_vel_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
            sub_amcl_pose_ = node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose",10, std::bind(&LocalizationInit::amclCallback,this,_1));

            node_->declare_parameter("theshold_covariance",0.7);
            node_->declare_parameter("vel_turn",0.4);

            covariance_ = std::numeric_limits<float>::max();


            theshold_covariance_ = node_->get_parameter("theshold_covariance").as_double();
            vel_turn_ = node_->get_parameter("vel_turn").as_double();
        }


        virtual BT::NodeStatus tick() override
        {
            // Waiting for service /save
            while (!client_->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for service /reinitialize_global_localization.");
                    return BT::NodeStatus::FAILURE;
                }
                RCLCPP_INFO(node_->get_logger(), "Waiting for service /reinitialize_global_localization to appear...");
            }
          

            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto result_future = client_->async_send_request(request);
            if (rclcpp::spin_until_future_complete(node_, result_future) != rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_ERROR(node_->get_logger(), "Unable to call /reinitialize_global_localization");
                return BT::NodeStatus::FAILURE;
            }
            else{
                turn_robot();
                return BT::NodeStatus::SUCCESS;
            }
        }

    private:
        double theshold_covariance_;
        float vel_turn_;
        float covariance_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_amcl_pose_;

        void turn_robot(){
            rclcpp::Rate rate(20);
            geometry_msgs::msg::Twist cmd_vel_msg;
            cmd_vel_msg.angular.z = vel_turn_;
            cmd_vel_msg.linear.x = 0.0;
            while( covariance_>theshold_covariance_ && rclcpp::ok()){
                pub_cmd_vel_->publish(cmd_vel_msg);
                RCLCPP_DEBUG(node_->get_logger(),"Turn robot");
                rclcpp::spin_some(node_);
                rate.sleep();
            }
            cmd_vel_msg.angular.z = 0.0;
            cmd_vel_msg.linear.x = 0.0;
            pub_cmd_vel_->publish(cmd_vel_msg);
            RCLCPP_DEBUG(node_->get_logger(),"Stop robot");

        }
        void amclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose){

            covariance_ = (pose->pose.covariance[0] + pose->pose.covariance[7] + pose->pose.covariance[35])/3;
            RCLCPP_DEBUG(node_->get_logger(),"Recivied amcl pose");
        }
};