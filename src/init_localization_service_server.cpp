#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rb1_shelf_msgs/srv/init_localization.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#define PI 3.141592653589793238462643
using initLocSrv = rb1_shelf_msgs::srv::InitLocalization;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class InitLocalizationServer : public rclcpp::Node {
public:
  InitLocalizationServer() : Node("init_localization_server") {

    // transfor broadcaster
    broadcaster_1 = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // services
    srv_ = create_service<initLocSrv>(
        "/init_localization_server",
        std::bind(&InitLocalizationServer::initLocalizationCallback, this, _1,
                  _2));

    t.transform.translation.z = std::numeric_limits<double>::max();

    // Parametros
    this->declare_parameter("pose_map_to_station_charge_x", 0.0);
    pose_map_to_station_charge_x_ =
        this->get_parameter("pose_map_to_station_charge_x").as_double();
    RCLCPP_INFO(this->get_logger(), "Pose map to station charge x [%.3f] ",
                pose_map_to_station_charge_x_);

    // Parametros
    this->declare_parameter("pose_map_to_station_charge_y", 0.0);
    pose_map_to_station_charge_y_ =
        this->get_parameter("pose_map_to_station_charge_y").as_double();
    RCLCPP_INFO(this->get_logger(), "Pose map to station charge y [%.3f] ",
                pose_map_to_station_charge_x_);

    // Parametros
    this->declare_parameter("pose_map_to_station_charge_yaw", 0.0);
    pose_map_to_station_charge_yaw_ =
        this->get_parameter("pose_map_to_station_charge_yaw").as_double();
    RCLCPP_INFO(this->get_logger(), "Pose map to station charge yaw [%.3f] ",
                pose_map_to_station_charge_yaw_);

  pose_map_to_station_charge_x:
  pose_map_to_station_charge_y:
  pose_map_to_station_charge_yaw:

    RCLCPP_INFO(this->get_logger(),
                "Server de servidor [init_localization_server] inicializado ");
  }

private:
  rclcpp::Service<initLocSrv>::SharedPtr srv_;

  geometry_msgs::msg::TransformStamped t;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_1;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // params
  double pose_map_to_station_charge_x_;
  double pose_map_to_station_charge_y_;
  double pose_map_to_station_charge_yaw_;

  // pose charge estation
  geometry_msgs::msg::Point pose_charte_station_;

  // services callback
  void findShelfCallback(const std::shared_ptr<initLocSrv::Request> request,
                         const std::shared_ptr<initLocSrv::Response> response) {
    pose_charte_station_ = *request;
    RCLCPP_DEBUG(this->get_logger(),
                 "-------------------------------------------");
    calculate_transform_station_charge();

    RCLCPP_DEBUG(this->get_logger(),
                 "-------------------------------------------");
  }

  void calculate_transform_station_charge() {
    // init transform variable
    t.transform.translation.z = std::numeric_limits<double>::max();

    float ptr_data[3];
    ptr_data[0] = pose_charte_station_.x;
    ptr_data[1] = pose_charte_station_.y;
    ptr_data[2] = pose_charte_station_.z;

    RCLCPP_DEBUG(node_->get_logger(), "dt [%.3f]", ptr_data[0]);
    RCLCPP_DEBUG(node_->get_logger(), "theta_total [%.3f]", ptr_data[1]);
    float x;
    float y;
    float z;
    double roll, pitch, yaw;

    rclcpp::Rate rate(20);
    while (t.transform.translation.z == std::numeric_limits<double>::max() &&
           rclcpp::ok()) {
      try {
        RCLCPP_ERROR(node_->get_logger(), "Error al obtener la transformacion");
        t = tf_buffer_->lookupTransform("robot_base_link",
                                        "robot_front_laser_base_link",
                                        tf2::TimePointZero);
        RCLCPP_ERROR(node_->get_logger(), "Llame a lookupTransform");
      } catch (tf2::TransformException &ex) {
      }
      rclcpp::spin_some(node_);
      rate.sleep();
    }
    RCLCPP_DEBUG(node_->get_logger(), "Se obtuvo la transformacion");
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
      RCLCPP_DEBUG(node_->get_logger(), "Transform from frame1 to frame2:");
      RCLCPP_DEBUG(node_->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f",
                   x, y, z);
      RCLCPP_DEBUG(node_->get_logger(),
                   "Rotation: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch,
                   yaw);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get transform: %s",
                   ex.what());
    }

    geometry_msgs::msg::TransformStamped transform_;
    transform_.header.frame_id =
        "robot_base_link"; // Marco de referencia fuente
    transform_.child_frame_id =
        "charge_station"; // Marco de referencia objetivo
    transform_.header.stamp = node_->get_clock()->now();
    // Definir la transformación estática (traslación y rotación)
    /*transform_.transform.translation.x = 0;
    transform_.transform.translation.y = 0;*/
    transform_.transform.translation.x =
        x + ptr_data[0] * std::sin(ptr_data[1]) * std::sin(yaw) +
        ptr_data[0] * std::cos(ptr_data[1]) * std::cos(yaw);
    transform_.transform.translation.y =
        y + ptr_data[0] * std::cos(ptr_data[1]) * std::sin(yaw) -
        ptr_data[0] * std::sin(ptr_data[1]) * std::cos(yaw);

    transform_.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(roll, pitch - M_PI, yaw - ptr_data[2] + M_PI); // Sin rotación

    tf2::Quaternion rotation_quat;
    rotation_quat.setRPY(0, M_PI, 0); // 90 degrees in radians
    tf2::Quaternion new_quat = rotation_quat * q;

    // Normalize the resulting quaternion
    new_quat.normalize();

    transform_.transform.rotation.x = q.x();
    transform_.transform.rotation.y = q.y();
    transform_.transform.rotation.z = q.z();
    transform_.transform.rotation.w = q.w();
    // Publicar la transformación estática
    broadcaster_1->sendTransform(transform_);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Transform traslation  x  [%.2f] y  [%.2f] z  [%.2f] ",
                 transform_.transform.translation.x,
                 transform_.transform.translation.y,
                 transform_.transform.translation.z);
    RCLCPP_DEBUG(node_->get_logger(),
                 "Se publico correctamente el frame [charge_station]  !!!");
    // hallo el angulo de transformacion

    // hallamos la matriz inversa
  }

  int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    // Instantiate a node.
    rclcpp::Node::SharedPtr node = std::make_shared<findShelfServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
  }