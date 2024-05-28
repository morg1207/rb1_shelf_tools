#pragma once

#include <functional>
#include <limits>
#include <unistd.h> // Used by sleep
#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>

#include "geometry_msgs/msg/point.hpp"

#include <behaviortree_cpp/action_node.h>

using namespace std::placeholders;


class PublishTransform : public BT::SyncActionNode
{
    public:
        PublishTransform(const std::string& name, const BT::NodeConfiguration& config )
            : BT::SyncActionNode(name, config)
        {
            node_ = rclcpp::Node::make_shared("publish_transform_node");

            // transfor broadcaster
            broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
            
            t.transform.translation.z = std::numeric_limits<double>::max();

            //params
            node_->declare_parameter("distance_aproach_shelf",0.4);
            distance_aproach_shelf_ = node_->get_parameter("distance_aproach_shelf").as_double();

            
        }

        static BT::PortsList providedPorts()
        {
            return{ 
                BT::InputPort<geometry_msgs::msg::Point>("position_shelf"),
                BT::OutputPort<geometry_msgs::msg::Pose>("target_goal")
                
                };
        }

        virtual BT::NodeStatus tick() override
        {
            if (!getInput<geometry_msgs::msg::Point>("position_shelf", position_shelf_)) {
                // if I can't get this, there is something wrong with your BT.
                // For this reason throw an exception instead of returning FAILURE
                throw BT::RuntimeError("missing required input [position_shelf]");
            }
            publish_transfor();
            publish_target_goal();
            return BT::NodeStatus::SUCCESS;

        }

    private:

        rclcpp::Node::SharedPtr node_;
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        geometry_msgs::msg::Point position_shelf_;
        geometry_msgs::msg::TransformStamped t;

        double distance_aproach_shelf_;

        void publish_transfor() {
            // init transform variable
            t.transform.translation.z = std::numeric_limits<double>::max();

            float ptr_data[3];
            ptr_data[0] =  position_shelf_.x;
            ptr_data[1] =  position_shelf_.y;
            ptr_data[2] =  position_shelf_.z;

            RCLCPP_INFO(node_->get_logger(), "dt [%.3f]", ptr_data[0]);
            RCLCPP_INFO(node_->get_logger(), "theta_total [%.3f]", ptr_data[1]);
            float x;
            float y;
            float z;
            double roll, pitch, yaw;

            rclcpp::Rate rate(20);
            while(t.transform.translation.z  == std::numeric_limits<double>::max() && rclcpp::ok()){
                try {
                    RCLCPP_ERROR(node_->get_logger(),"Error al obtener la transformacion");
                    t = tf_buffer_->lookupTransform("robot_odom", "robot_front_laser_base_link",tf2::TimePointZero);
                    RCLCPP_ERROR(node_->get_logger(),"Llame a lookupTransform");
                }catch (tf2::TransformException &ex) {
                }
                rclcpp::spin_some(node_);
                rate.sleep();
            }
            RCLCPP_INFO(node_->get_logger(),"Se obtuvo la transformacion");
            try {
                x = t.transform.translation.x;
                y = t.transform.translation.y;
                z = t.transform.translation.z;
                // Obtener los valores de traslación y rotación

                tf2::Quaternion quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                            t.transform.rotation.z,
                                            t.transform.rotation.w);
                tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

                // Imprimir los valores obtenidos
                RCLCPP_INFO(node_->get_logger(), "Transform from frame1 to frame2:");
                RCLCPP_INFO(node_->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f", x,
                            y, z);
                RCLCPP_INFO(node_->get_logger(),
                            "Rotation: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch,
                            yaw);
            } catch (tf2::TransformException &ex) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get transform: %s",
                            ex.what());
            }

            geometry_msgs::msg::TransformStamped transform_;
            transform_.header.frame_id = "robot_odom"; // Marco de referencia fuente
            transform_.child_frame_id = "cart_frame";  // Marco de referencia objetivo
            transform_.header.stamp = node_->get_clock()->now();
            // Definir la transformación estática (traslación y rotación)
            /*transform_.transform.translation.x = 0;
            transform_.transform.translation.y = 0;*/
            transform_.transform.translation.x =
                x + ptr_data[0] * std::sin(ptr_data[1]) * std::sin(yaw) +  ptr_data[0] * std::cos(ptr_data[1]) * std::cos(yaw) ;
            transform_.transform.translation.y =
                y + ptr_data[0] * std::cos(ptr_data[1])* std::sin(yaw)  - ptr_data[0] * std::sin(ptr_data[1])* std::cos(yaw);

            transform_.transform.translation.z = z;

           
            tf2::Quaternion q;
            q.setRPY(roll , pitch -M_PI, yaw - ptr_data[2] +M_PI); // Sin rotación


            tf2::Quaternion rotation_quat;
            rotation_quat.setRPY(0, M_PI, 0);  // 90 degrees in radians
            tf2::Quaternion new_quat = rotation_quat * q;

            // Normalize the resulting quaternion
            new_quat.normalize();

            transform_.transform.rotation.x = q.x();
            transform_.transform.rotation.y = q.y();
            transform_.transform.rotation.z = q.z();
            transform_.transform.rotation.w = q.w();
            // Publicar la transformación estática
            broadcaster_->sendTransform(transform_);

            RCLCPP_INFO(node_->get_logger(), "Transform traslation  x  [%.2f] y  [%.2f] z  [%.2f] ", transform_.transform.translation.x ,transform_.transform.translation.y ,transform_.transform.translation.z );
            RCLCPP_INFO(node_->get_logger(),"Se publico correctamente el frame [cart_frame]  !!!");
        }

        void publish_target_goal(){

            geometry_msgs::msg::TransformStamped transform_;
            transform_.header.frame_id = "cart_frame"; // Marco de referencia fuente
            transform_.child_frame_id = "target_goal_frame";  // Marco de referencia objetivo
            transform_.header.stamp = node_->get_clock()->now();

            transform_.transform.translation.x = -distance_aproach_shelf_;
            transform_.transform.translation.y = 0;
            transform_.transform.translation.z = 0;
            transform_.transform.rotation.x = 0;
            transform_.transform.rotation.y = 0;
            transform_.transform.rotation.z = 0;
            transform_.transform.rotation.w = 1;
           
             // Publicar la transformación estática
            broadcaster_->sendTransform(transform_);

            RCLCPP_INFO(node_->get_logger(), "Transform cart_frame to target_goal_frame traslation  x  [%.2f] y  [%.2f] z  [%.2f] ", transform_.transform.translation.x ,transform_.transform.translation.y ,transform_.transform.translation.z );
            RCLCPP_INFO(node_->get_logger(),"Se publico correctamente el frame [target_goal_frame] !!!");

             // init transform variable
            t.transform.translation.z = std::numeric_limits<double>::max();
            rclcpp::Rate rate(20);
            while(t.transform.translation.z  == std::numeric_limits<double>::max() && rclcpp::ok()){
                try {
                    RCLCPP_ERROR(node_->get_logger(),"Error al obtener la transformacion de [%s]  a [%s]", "map","target_goal_frame");
                    t = tf_buffer_->lookupTransform("map", "target_goal_frame",tf2::TimePointZero);
                    RCLCPP_ERROR(node_->get_logger(),"Llame a lookupTransform");
                }catch (tf2::TransformException &ex) {
                }
                rclcpp::spin_some(node_);
                rate.sleep();
            }
            RCLCPP_INFO(node_->get_logger(),"Se obtuvo la transformacion de map a target_goal_frame");
            
            geometry_msgs::msg::Pose target_goal;
            target_goal.position.x = t.transform.translation.x;
            target_goal.position.y = t.transform.translation.y;
            target_goal.position.z = t.transform.translation.z;
            target_goal.orientation.x = t.transform.rotation.x;
            target_goal.orientation.y = t.transform.rotation.y;
            target_goal.orientation.z = t.transform.rotation.z;
            target_goal.orientation.w = t.transform.rotation.w;
   
            RCLCPP_INFO(node_->get_logger(), "Transform traslation map a target_goal x  [%.2f] y  [%.2f] z  [%.2f]  w[%.2f]", target_goal.position.x,target_goal.position.y,target_goal.orientation.z,target_goal.orientation.w);
            setOutput<geometry_msgs::msg::Pose>("target_goal",target_goal);

        }


};