#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rb1_shelf_msgs/srv/approach_shelf.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>

using approachShelfSrv = rb1_shelf_msgs::srv::ApproachShelf;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

enum class ControlState {
  INIT,
  CONTROL_APPROACH,
  CONTROL_DIRECTION1,
  CONTROL_DIRECTION2,
  CONTROL_SHELF,
  CONTROL_APPROACH_END,
  ENTER_TO_SHELF,
  BACK_TO_SHELF,
  END,
};

class approachShelfServer : public rclcpp::Node {
public:
  approachShelfServer() : Node("approach_shelf_server") {
    srv_approach_ = create_service<approachShelfSrv>(
        "/approach_shelf_server",
        std::bind(&approachShelfServer::approachShelfCallback, this, _1, _2));
    pub_cmd_vel_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    t.transform.translation.z = std::numeric_limits<double>::max();

    this->declare_parameter("distance_approach_target_error", 0.5);
    distance_approach_target_error_ =
        this->get_parameter("distance_approach_target_error").as_double();

    this->declare_parameter("distance_approach_target_error_back", 0.05);
    distance_approach_target_error_back_ =
        this->get_parameter("distance_approach_target_error_back").as_double();

    this->declare_parameter("angle_approach_target_error", 0.5);
    angle_approach_target_error_ =
        this->get_parameter("angle_approach_target_error").as_double();

    this->declare_parameter("vel_min_linear_x", 0.5);
    vel_min_linear_x_ = this->get_parameter("vel_min_linear_x").as_double();

    this->declare_parameter("vel_min_angular_z", 0.5);
    vel_min_angular_z_ = this->get_parameter("vel_min_angular_z").as_double();

    this->declare_parameter("vel_max_linear_x", 0.5);
    vel_max_linear_x_ = this->get_parameter("vel_max_linear_x").as_double();

    this->declare_parameter("vel_max_angular_z", 0.5);
    vel_max_angular_z_ = this->get_parameter("vel_max_angular_z").as_double();

    this->declare_parameter("kp_angular", 0.5);
    kp_angular_ = this->get_parameter("kp_angular").as_double();

    this->declare_parameter("kp_lineal", 0.5);
    kp_lineal_ = this->get_parameter("kp_lineal").as_double();

    this->declare_parameter("laser_min_range", 0.5);
    laser_min_range_ = this->get_parameter("laser_min_range").as_double();

    RCLCPP_INFO(this->get_logger(),
                "Servidor de servicio [approach_shelf_server] inicializado ");

    RCLCPP_INFO(this->get_logger(), "Vel min lin x [%.3f] ", vel_min_linear_x_);
    RCLCPP_INFO(this->get_logger(), "Vel min angular z [%.3f] ",
                vel_min_angular_z_);
    RCLCPP_INFO(this->get_logger(), "Vel max lin x [%.3f] ", vel_max_linear_x_);
    RCLCPP_INFO(this->get_logger(), "Vel max angular z [%.3f] ",
                vel_max_angular_z_);

    RCLCPP_INFO(this->get_logger(), "Kp lineal [%.3f] ", kp_lineal_);
    RCLCPP_INFO(this->get_logger(), "Kp angular [%.3f] ", kp_angular_);

    RCLCPP_INFO(this->get_logger(), "Distance to target_goal error [%.3f] ",
                distance_approach_target_error_);
    RCLCPP_INFO(this->get_logger(), "Angle to target_goal error [%.3f] ",
                angle_approach_target_error_);
    RCLCPP_INFO(this->get_logger(), "Laser min range [%.3f] ",
                laser_min_range_);
    RCLCPP_INFO(this->get_logger(),
                "Shelf approach control server initialized");
  }

private:
  rclcpp::Service<approachShelfSrv>::SharedPtr srv_approach_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;

  geometry_msgs::msg::Twist msg_cmd_vel;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::TimerBase::SharedPtr timer_control_;
  // variables
  float distance_approach_target_error_;
  float angle_approach_target_error_;
  float vel_min_linear_x_;
  float vel_min_angular_z_;
  float vel_max_linear_x_;
  float vel_max_angular_z_;
  float kp_angular_;
  float kp_lineal_;
  float laser_min_range_;
  float distance_approach_target_error_back_;

  // Control variables
  float distance_target;
  float theta_target;
  double yaw_target_;

  // tyepe control
  std::string type_control_;

  ControlState control_state = ControlState::INIT;

  geometry_msgs::msg::TransformStamped t;

  void approachShelfCallback(
      const std::shared_ptr<approachShelfSrv::Request> request,
      const std::shared_ptr<approachShelfSrv::Response> response) {
    type_control_ = request->type_control;
    controlRobot();
    if (control_state == ControlState::END) {

      response->state_control_output = "END";
      control_state = ControlState::INIT;
    } else {
      response->state_control_output = "RUNING";
    }
    response->success = true;
  }
  void init() { control_state = ControlState::INIT; }
  void getTransform(std::string frame_head, std::string frame_child) {

    t.transform.translation.z = std::numeric_limits<double>::max();
    rclcpp::Rate rate(20);
    while (t.transform.translation.z == std::numeric_limits<double>::max() &&
           rclcpp::ok()) {
      try {
        t = tf_buffer_->lookupTransform(frame_head, frame_child,
                                        tf2::TimePointZero);

      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Llame a lookupTransform");
        RCLCPP_ERROR(this->get_logger(),
                     "Error al obtener la transformacion de [%s]  a [%s]",
                     frame_head.c_str(), frame_child.c_str());
      }
      RCLCPP_DEBUG(this->get_logger(), "Se obtuvo la transformacion");
      rate.sleep();
    }
  }

  void calculateErrors() {

    distance_target = sqrt(std::pow(t.transform.translation.x, 2) +
                           std::pow(t.transform.translation.y, 2));
    theta_target =
        std::atan2(t.transform.translation.y, t.transform.translation.x);

    double roll, pitch;
    // Hallar el angulo target de direccion
    tf2::Quaternion quaternion(t.transform.rotation.x, t.transform.rotation.y,
                               t.transform.rotation.z, t.transform.rotation.w);
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw_target_);
  }

  void controlRobot() {

    RCLCPP_INFO(this->get_logger(), "Type control [%s]", type_control_.c_str());
    if (type_control_ == "approach_shelf") {
      switch (control_state) {
      case ControlState::INIT:
        control_state = ControlState::CONTROL_DIRECTION1;
        RCLCPP_INFO(this->get_logger(), "State control [CONTROL_INIT]");
        break;
      // Control para mirar primero al punto de apollo y evitar las trayectorias
      // curvas muy cerradas
      case ControlState::CONTROL_DIRECTION1:
        getTransform("robot_base_link", "target_goal_frame");
        calculateErrors();
        if (abs(theta_target) < angle_approach_target_error_) {
          control_state = ControlState::CONTROL_APPROACH;
          robotStop();
        }
        RCLCPP_INFO(this->get_logger(), "State control [DIRECCTION1]");
        msg_cmd_vel.angular.z = theta_target * kp_angular_;
        msg_cmd_vel.linear.x = 0.0;
        msg_cmd_vel.angular.z = saturateVel(
            msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
        pub_cmd_vel_->publish(msg_cmd_vel);
        break;

      case ControlState::CONTROL_APPROACH:
        getTransform("robot_base_link", "target_goal_frame");
        calculateErrors();
        if (distance_target < distance_approach_target_error_) {
          control_state = ControlState::CONTROL_DIRECTION2;
          robotStop();
        }

        RCLCPP_INFO(this->get_logger(), "State control [CONTROL_APPROACH]");
        msg_cmd_vel.angular.z = theta_target * kp_angular_;
        msg_cmd_vel.linear.x = distance_target * kp_lineal_;
        msg_cmd_vel.linear.x = saturateVel(
            msg_cmd_vel.linear.x, vel_min_linear_x_, vel_max_linear_x_);
        msg_cmd_vel.angular.z = saturateVel(
            msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
        pub_cmd_vel_->publish(msg_cmd_vel);

        break;

      case ControlState::CONTROL_DIRECTION2:
        getTransform("robot_base_link", "target_goal_frame");
        calculateErrors();
        if (abs(yaw_target_) < angle_approach_target_error_) {
          control_state = ControlState::CONTROL_APPROACH_END;
          robotStop();
        }
        RCLCPP_INFO(this->get_logger(), "State control [DIRECCTION2]");
        msg_cmd_vel.angular.z = yaw_target_ * kp_angular_;
        msg_cmd_vel.linear.x = 0.0;
        msg_cmd_vel.angular.z = saturateVel(
            msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
        pub_cmd_vel_->publish(msg_cmd_vel);
        break;

      case ControlState::CONTROL_APPROACH_END:
        getTransform("robot_base_link", "cart_frame");
        calculateErrors();
        if (distance_target < laser_min_range_) {
          control_state = ControlState::END;
          robotStop();
        }
        RCLCPP_INFO(this->get_logger(), "State control [CONTROL_APPROACH_END]");
        msg_cmd_vel.angular.z = theta_target * kp_angular_;
        msg_cmd_vel.linear.x = distance_target * kp_lineal_;
        msg_cmd_vel.linear.x = saturateVel(
            msg_cmd_vel.linear.x, vel_min_linear_x_, vel_max_linear_x_);
        msg_cmd_vel.angular.z = saturateVel(
            msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
        pub_cmd_vel_->publish(msg_cmd_vel);

        break;

      default:
        RCLCPP_INFO(this->get_logger(), "estate default");
        control_state = ControlState::INIT;
        break;
      }
    } else if (type_control_ == "enter_to_shelf") {
      switch (control_state) {
      case ControlState::INIT:
        control_state = ControlState::ENTER_TO_SHELF;
        RCLCPP_INFO(this->get_logger(), "State control [CONTROL_INIT]");
        break;

      case ControlState::ENTER_TO_SHELF:
        getTransform("robot_base_link", "target_shelf_frame");
        calculateErrors();
        if (distance_target < distance_approach_target_error_) {
          control_state = ControlState::END;
          robotStop();
        }
        RCLCPP_INFO(this->get_logger(), "distance target [%.3f]",
                    distance_target);
        RCLCPP_INFO(this->get_logger(), "State control [ENTER_TO_SHELF]");
        msg_cmd_vel.angular.z = theta_target * kp_angular_;
        msg_cmd_vel.linear.x = distance_target * kp_lineal_;
        msg_cmd_vel.linear.x = saturateVel(
            msg_cmd_vel.linear.x, vel_min_linear_x_, vel_max_linear_x_);
        msg_cmd_vel.angular.z = saturateVel(
            msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
        pub_cmd_vel_->publish(msg_cmd_vel);

        break;

      default:
        control_state = ControlState::INIT;
        break;
      }
    } else if (type_control_ == "back_to_shelf") {
      switch (control_state) {
      case ControlState::INIT:
        control_state = ControlState::BACK_TO_SHELF;
        RCLCPP_INFO(this->get_logger(), "State control [CONTROL_INIT]");
        break;

      case ControlState::BACK_TO_SHELF:
        getTransform("robot_base_link", "back_frame");
        calculateErrors();
        if (distance_target < distance_approach_target_error_back_) {
          // if (distance_target <) {
          control_state = ControlState::END;
          robotStop();
        }
        if (theta_target > 0) {
          theta_target = theta_target - M_PI;
        } else {
          theta_target = M_PI - theta_target;
        }
        RCLCPP_INFO(this->get_logger(), "Error distace [%.3f]",
                    distance_target);
        RCLCPP_INFO(this->get_logger(), "theta_target [%.3f]", theta_target);
        RCLCPP_INFO(this->get_logger(), "State control [BACK_TO_SHELF]");
        msg_cmd_vel.angular.z = 0.0;
        msg_cmd_vel.angular.z = 0.0;
        msg_cmd_vel.linear.x = distance_target * kp_lineal_;
        msg_cmd_vel.linear.x = saturateVel(
            msg_cmd_vel.linear.x, vel_min_linear_x_, vel_max_linear_x_);
        msg_cmd_vel.angular.z = saturateVel(
            msg_cmd_vel.angular.z, vel_min_angular_z_, vel_max_angular_z_);
        // retroceso
        msg_cmd_vel.linear.x = msg_cmd_vel.linear.x;
        pub_cmd_vel_->publish(msg_cmd_vel);

        break;

      default:
        control_state = ControlState::INIT;
        break;
      }
    }
  }
  void robotStop() {
    msg_cmd_vel.angular.z = 0.0;
    msg_cmd_vel.linear.x = 0.0;
    pub_cmd_vel_->publish(msg_cmd_vel);
  }
  float saturateVel(float vel, float vel_lower_limit, float vel_upper_limit) {
    if (vel > 0) {
      return std::max(vel_lower_limit, std::min(vel, vel_upper_limit));
    } else {
      vel = std::max(vel_lower_limit, std::min(-vel, vel_upper_limit));
      return -vel;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Instantiate a node.
  rclcpp::Node::SharedPtr node = std::make_shared<approachShelfServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
