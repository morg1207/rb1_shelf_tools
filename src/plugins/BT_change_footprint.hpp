#pragma once

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/point32.hpp"

#include <behaviortree_cpp/action_node.h>


using namespace std::placeholders;


class ChangeFootprint : public BT::SyncActionNode
{
    public:
        ChangeFootprint(const std::string& name, const BT::NodeConfiguration& config )
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("change_footprint");
            pub_local_footprint_ = node_->create_publisher<geometry_msgs::msg::Polygon>("/local_costmap/footprint",10);
            pub_global_footprint_ = node_->create_publisher<geometry_msgs::msg::Polygon>("/global_costmap/footprint",10);
            
            //params
            node_->declare_parameter("shelf_length",1.0);
            shelf_length_ = node_->get_parameter("shelf_length").as_double();

            node_->declare_parameter("shelf_width",0.8);
            shelf_width_ = node_->get_parameter("shelf_width").as_double();

            node_->declare_parameter("rb1_radius",0.25);
            rb1_radius_ = node_->get_parameter("rb1_radius").as_double();


            RCLCPP_INFO(node_->get_logger(), "shelf length [%.3f] ",shelf_length_);
            RCLCPP_INFO(node_->get_logger(), "shelf width [%.3f] ",shelf_width_);
            RCLCPP_INFO(node_->get_logger(), "radius rb1 [%.3f] ",rb1_radius_);



        }
        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<std::string>("change_footprint_action")};
        }

        virtual BT::NodeStatus tick() override
        {
            std::string change_footprint_action;

            if (!getInput<std::string>("change_footprint_action", change_footprint_action)) {
                // if I can't get this, there is something wrong with your BT.
                // For this reason throw an exception instead of returning FAILURE
                throw BT::RuntimeError("missing required input [change_footprint_action]");
            }
            RCLCPP_INFO(node_->get_logger(), "footprint action: %s", change_footprint_action.c_str());


            if (change_footprint_action == "with_shelf")
            {  
                geometry_msgs::msg::Polygon poly_msg;
                poly_msg.points.resize(4); // Ensure there are 4 points
                geometry_msgs::msg::Point32 point_msg;
                point_msg.x = shelf_length_/2;
                point_msg.y = shelf_width_/2;
                point_msg.z = 0.0;
                poly_msg.points[0] = point_msg;
                point_msg.x = shelf_length_/2;
                point_msg.y = -shelf_width_/2;
                point_msg.z = 0.0;
                poly_msg.points[1] = point_msg;
                point_msg.x = -shelf_length_/2;
                point_msg.y = -shelf_width_/2;
                point_msg.z = 0.0;
                poly_msg.points[2] = point_msg;
                point_msg.x = -shelf_length_/2;
                point_msg.y = shelf_width_/2;
                point_msg.z = 0.0;
                poly_msg.points[3] = point_msg;
                rclcpp::Rate rate(10);
                int i=0;
                while(i<10){
                    pub_local_footprint_->publish(poly_msg);
                    pub_global_footprint_->publish(poly_msg);
                    RCLCPP_INFO(node_->get_logger(), "Publish footprint  [%s]", change_footprint_action.c_str());
                    i++;
                    rate.sleep();

                }
                return BT::NodeStatus::SUCCESS;
            }
            else if (change_footprint_action == "without_shelf")
            {
                geometry_msgs::msg::Polygon poly_msg;
                poly_msg.points.resize(4); // Ensure there are 4 points
                geometry_msgs::msg::Point32 point_msg;
                point_msg.x = rb1_radius_;
                point_msg.y = rb1_radius_;
                point_msg.z = 0.0;
                poly_msg.points[0] = point_msg;
                point_msg.x = rb1_radius_;
                point_msg.y = -rb1_radius_;
                point_msg.z = 0.0;
                poly_msg.points[1] = point_msg;
                point_msg.x = -rb1_radius_;
                point_msg.y = -rb1_radius_;
                point_msg.z = 0.0;
                poly_msg.points[2] = point_msg;
                point_msg.x = -rb1_radius_;
                point_msg.y = rb1_radius_;
                point_msg.z = 0.0;
                poly_msg.points[3] = point_msg;
                pub_local_footprint_->publish(poly_msg);
                pub_global_footprint_->publish(poly_msg);
                 RCLCPP_INFO(node_->get_logger(), "Publish footprint  [%s]", change_footprint_action.c_str());
                return BT::NodeStatus::SUCCESS;
            }
            else
            {
                RCLCPP_ERROR(node_->get_logger(), "Unknown action: %s", change_footprint_action.c_str());
                return BT::NodeStatus::FAILURE;
            }
        }
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_local_footprint_;
        rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr pub_global_footprint_;
        double shelf_length_;
        double shelf_width_;
        double rb1_radius_;

    };
