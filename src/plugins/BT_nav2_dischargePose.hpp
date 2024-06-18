#pragma once

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include <behaviortree_cpp/action_node.h>


using namespace std::placeholders;


class Nav2DischargePose : public BT::SyncActionNode
{
    public:
        Nav2DischargePose(const std::string& name, const BT::NodeConfiguration& config )
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("pose_discharge_pose");

            sub_pose_nav_ = node_->create_subscription<geometry_msgs::msg::Pose>("/pose_discharge_nav",10,std::bind(&Nav2DischargePose::callPoseNav,this,_1));
            
        }
        static BT::PortsList providedPorts()
        {
            return{ 
                BT::OutputPort<geometry_msgs::msg::Pose>("pose_discharge_nav"),
                BT::InputPort<int>("time_for_wait")
                };
        }

        virtual BT::NodeStatus tick() override
        {
            auto start_time = node_->now();
            if (!getInput<int>("time_for_wait",
                                             time_for_wait_)) {
                // if I can't get this, there is something wrong with your BT.
                // For this reason throw an exception instead of returning FAILURE
                throw BT::RuntimeError("missing required input [time_for_wait]");
            }
            rclcpp::Rate rate(10);

            wait_duration_ = std::chrono::seconds(time_for_wait_);
            auto end_time = start_time + wait_duration_;

            while (pose_recivided_ != true && rclcpp::ok() &&  node_->now() < end_time ){
                rclcpp::spin_some(node_);
                RCLCPP_INFO(node_->get_logger(), "Wait for pose for nav");
                rate.sleep();
            }
            if (pose_recivided_){
                setOutput<geometry_msgs::msg::Pose>("pose_discharge_nav",pose_nav);
                return BT::NodeStatus::SUCCESS;
            }
            else{
                RCLCPP_ERROR(node_->get_logger(), "Pose no recivida en [%d]",time_for_wait_ );
                return BT::NodeStatus::FAILURE;
            }
        }

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr sub_pose_nav_;
        geometry_msgs::msg::Pose pose_nav; 
        std::chrono::seconds wait_duration_;
        int time_for_wait_;
        bool pose_recivided_;
        void callPoseNav(const geometry_msgs::msg::Pose::SharedPtr pose){
                pose_recivided_ = true;
                pose_nav = *pose;
            
        }

    };
