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

struct TransformationMatrix2D {
  double m[3][3];

  void print() const {
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        std::cout << m[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
};

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
  void initLocalizationCallback(
      const std::shared_ptr<initLocSrv::Request> request,
      const std::shared_ptr<initLocSrv::Response> response) {
    pose_charte_station_ = request->station_position;
    RCLCPP_DEBUG(this->get_logger(),
                 "-------------------------------------------");
    calculate_transform_station_charge();

    RCLCPP_DEBUG(this->get_logger(),
                 "-------------------------------------------");
    response->success = true;
  }

  void calculate_transform_station_charge() {
    // init transform variable
    t.transform.translation.z = std::numeric_limits<double>::max();

    float ptr_data[3];
    ptr_data[0] = pose_charte_station_.x;
    ptr_data[1] = pose_charte_station_.y;
    ptr_data[2] = pose_charte_station_.z;

    RCLCPP_DEBUG(this->get_logger(), "dt [%.3f]", ptr_data[0]);
    RCLCPP_DEBUG(this->get_logger(), "theta_total [%.3f]", ptr_data[1]);
    float x;
    float y;
    float z;
    double roll, pitch, yaw;

    rclcpp::Rate rate(20);
    while (t.transform.translation.z == std::numeric_limits<double>::max() &&
           rclcpp::ok()) {
      try {
        t = tf_buffer_->lookupTransform("robot_base_link",
                                        "robot_front_laser_base_link",
                                        tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Error al obtener la transformacion");
        RCLCPP_ERROR(this->get_logger(), "Llame a lookupTransform");
      }
      RCLCPP_DEBUG(this->get_logger(), "Se obtuvo la transformacion");

      rate.sleep();
    }
    RCLCPP_DEBUG(this->get_logger(), "Se obtuvo la transformacion");
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
      RCLCPP_DEBUG(this->get_logger(), "Transform from frame1 to frame2:");
      RCLCPP_DEBUG(this->get_logger(), "Translation: x=%.2f, y=%.2f, z=%.2f", x,
                   y, z);
      RCLCPP_DEBUG(this->get_logger(),
                   "Rotation: roll=%.2f, pitch=%.2f, yaw=%.2f", roll, pitch,
                   yaw);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s",
                   ex.what());
    }

    geometry_msgs::msg::TransformStamped transform_;
    transform_.header.frame_id =
        "robot_base_link"; // Marco de referencia fuente
    transform_.child_frame_id =
        "charge_station"; // Marco de referencia objetivo
    transform_.header.stamp = this->get_clock()->now();
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

    RCLCPP_DEBUG(this->get_logger(),
                 "Transform traslation  x  [%.2f] y  [%.2f] z  [%.2f] ",
                 transform_.transform.translation.x,
                 transform_.transform.translation.y,
                 transform_.transform.translation.z);
    RCLCPP_DEBUG(this->get_logger(),
                 "Se publico correctamente el frame [charge_station]  !!!");

    // pido la transformacion de charge estation a robot odom

    // hallo el angulo de transformacion
    float pose_station_x = transform_.transform.translation.x;
    float pose_station_y = transform_.transform.translation.y;
    float pose_station_yaw = yaw - ptr_data[2];
    RCLCPP_DEBUG(this->get_logger(),
                 "Yam del frame [charge_station] es [%.3f]!!!",
                 pose_station_yaw);

    TransformationMatrix2D matrix = createTransformationMatrix(
        pose_station_x, pose_station_y, pose_station_yaw);

    std::cout << "Matriz de transformación 2D:" << std::endl;
    matrix.print();

    TransformationMatrix2D inverseMatrix = invertTransformationMatrix(matrix);

    std::cout << "Matriz inversa de transformación 2D:" << std::endl;
    inverseMatrix.print();
    // resto los vectores robot - map
    float theta_robot =
        std::atan2(inverseMatrix.m[1][2], inverseMatrix.m[0][2]);
    RCLCPP_DEBUG(this->get_logger(), "Theta robot [%.3f]!!!", theta_robot);
    float theta_map = std::atan2(pose_map_to_station_charge_y_,
                                 pose_map_to_station_charge_x_);
    RCLCPP_DEBUG(this->get_logger(), "Theta map [%.3f]!!!", theta_map);
    float distance_robot =
        std::sqrt(std::pow(pose_station_x, 2) + std::pow(pose_station_y, 2));
    RCLCPP_DEBUG(this->get_logger(), "Distance robot [%.3f]!!!",
                 distance_robot);
    float distance_map = std::sqrt(std::pow(pose_map_to_station_charge_x_, 2) +
                                   std::pow(pose_map_to_station_charge_y_, 2));
    RCLCPP_DEBUG(this->get_logger(), "Distance map [%.3f]!!!", distance_map);
    float theta_final = theta_robot - theta_map;
    RCLCPP_DEBUG(this->get_logger(), "Theta final[%.3f]!!!", theta_final);
    float distance_final =
        std::sqrt(std::pow(distance_robot, 2) + std::pow(distance_map, 2) -
                  2 * distance_robot * distance_map * std::cos(theta_final));
    RCLCPP_DEBUG(this->get_logger(), "Distancia final [%.3f]!!!",
                 distance_final);
    // sumo como vectores i y j
    float x_end = distance_robot * std::cos(theta_robot) +
                  distance_map * std::cos(theta_map);
    float y_end = distance_robot * std::sin(theta_robot) +
                  distance_map * std::sin(theta_map);
    // theta
    float theta_direction_charge_robot = std::atan2(y_end, x_end);
    RCLCPP_DEBUG(this->get_logger(), "Theta direction final [%.3f]!!!",
                 theta_direction_charge_robot);
    // theta robot_map
    float theta_charge_robot =
        std::atan2(inverseMatrix.m[1][0], inverseMatrix.m[0][0]);
    RCLCPP_DEBUG(this->get_logger(), "Theta robot [%.3f]!!!",
                 theta_charge_robot);
    // Hallo el angulo demap robot base link

    // pose robot to map
  }
  TransformationMatrix2D createTransformationMatrix(double x, double y,
                                                    double theta) {
    TransformationMatrix2D matrix;

    matrix.m[0][0] = std::cos(theta);
    matrix.m[0][1] = -std::sin(theta);
    matrix.m[0][2] = x;

    matrix.m[1][0] = std::sin(theta);
    matrix.m[1][1] = std::cos(theta);
    matrix.m[1][2] = y;

    matrix.m[2][0] = 0;
    matrix.m[2][1] = 0;
    matrix.m[2][2] = 1;

    return matrix;
  }
  TransformationMatrix2D
  invertTransformationMatrix(const TransformationMatrix2D &matrix) {
    TransformationMatrix2D inverse;

    // Transponer la submatriz de rotación
    inverse.m[0][0] = matrix.m[0][0];
    inverse.m[0][1] = matrix.m[1][0];
    inverse.m[1][0] = matrix.m[0][1];
    inverse.m[1][1] = matrix.m[1][1];

    // Calcular la nueva traslación
    inverse.m[0][2] =
        -(matrix.m[0][0] * matrix.m[0][2] + matrix.m[1][0] * matrix.m[1][2]);
    inverse.m[1][2] =
        -(matrix.m[0][1] * matrix.m[0][2] + matrix.m[1][1] * matrix.m[1][2]);

    // La última fila sigue siendo la misma
    inverse.m[2][0] = 0;
    inverse.m[2][1] = 0;
    inverse.m[2][2] = 1;

    return inverse;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Instantiate a node.
  rclcpp::Node::SharedPtr node = std::make_shared<InitLocalizationServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}