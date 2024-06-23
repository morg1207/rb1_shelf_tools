#include "rclcpp/rclcpp.hpp"

#include "rb1_shelf_msgs/srv/find_nav_poses.hpp"

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

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
using findNavPoses = rb1_shelf_msgs::srv::FindNavPoses;

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

struct indexMap {
  int x_index;
  int y_index;
};

class FindPointForNav : public rclcpp::Node {
public:
  FindPointForNav() : Node("find_point_for_nav") {

    // transfor broadcaster
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // services
    srv_ = create_service<findNavPoses>(
        "/find_nav_poses",
        std::bind(&FindPointForNav::FindPointsForNavCallback, this, _1, _2));

    auto sensor_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // subcrition a mapa de costos
    sub_map_cost_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", sensor_qos,
        std::bind(&FindPointForNav::costMapCallback, this, _1));

    // inicio en un valor el contenido
    t.transform.translation.z = std::numeric_limits<double>::max();

    arrived_costmap = false;
    // parameteres
    this->declare_parameter("distance_to_shelf_for_find_pose", 0.5);
    distance_to_shelf_for_find_pose_ =
        this->get_parameter("distance_to_shelf_for_find_pose").as_double();
    RCLCPP_INFO(this->get_logger(), "distance to shelf for find pose [%.3f] ",
                distance_to_shelf_for_find_pose_);

    this->declare_parameter("resolution_for_find_nav_rad", 0.5);
    resolution_for_find_nav_rad_ =
        this->get_parameter("resolution_for_find_nav_rad").as_double();
    RCLCPP_INFO(this->get_logger(), "resolution for find nav rad [%.3f] ",
                resolution_for_find_nav_rad_);

    this->declare_parameter("angle_min_rad", -3.14);
    angle_min_rad_ = this->get_parameter("angle_min_rad").as_double();
    RCLCPP_INFO(this->get_logger(), "angle min for find [%.3f] ",
                angle_min_rad_);

    this->declare_parameter("angle_max_rad", 3.14);
    angle_max_rad_ = this->get_parameter("angle_max_rad").as_double();
    RCLCPP_INFO(this->get_logger(), "angle max for find [%.3f] ",
                angle_max_rad_);

    this->declare_parameter("cant_casileros_verify", 4);
    cant_casileros_verify_ =
        this->get_parameter("cant_casileros_verify").as_int();
    RCLCPP_INFO(this->get_logger(), "cant casileros verify [%d] ",
                cant_casileros_verify_);

    RCLCPP_INFO(this->get_logger(),
                "Navigation point search server initialized ");
  }

private:
  rclcpp::Service<findNavPoses>::SharedPtr srv_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_cost_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;

  geometry_msgs::msg::TransformStamped t;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::msg::Point point_nav_;
  float direction_;
  // parameters
  double distance_to_shelf_for_find_pose_;
  double resolution_for_find_nav_rad_;
  int cant_casileros_verify_;
  double angle_min_rad_;
  double angle_max_rad_;
  float robot_pose_x_;
  float robot_pose_y_;
  float shelf_pose_x_;
  float shelf_pose_y_;
  float module_;
  float u_y_;
  float u_x_;
  bool arrived_costmap;

  // laser callback
  void costMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    costmap_ = msg;
    arrived_costmap = true;
    RCLCPP_DEBUG(this->get_logger(), "Costmap arrived");
  }
  // services callback
  void FindPointsForNavCallback(
      const std::shared_ptr<findNavPoses::Request> request,
      const std::shared_ptr<findNavPoses::Response> response) {
    RCLCPP_DEBUG(this->get_logger(),
                 "-------------------------------------------");
    RCLCPP_DEBUG(this->get_logger(),
                 "Find point nav server for shelf_position x  [%.3f]  y [%.3f]",
                 request->shelf_position.x, request->shelf_position.y);
    shelf_pose_x_ = request->shelf_position.x;
    shelf_pose_y_ = request->shelf_position.y;
    direction_ = request->direction;

    // pido la transformacion del robot con respecto al map
    getTransform("map", "robot_base_link");

    RCLCPP_DEBUG(this->get_logger(), "passs calculate points");
    if (calculate_point()) {
      response->success = true;
      response->nav_position = point_nav_;
      RCLCPP_DEBUG(this->get_logger(), "Se calvulo un punto exitoso");
    } else {
      response->success = false;
    }
  }
  // funcion para encontrar la transformacion del robot con respecto al shelf
  bool calculate_point() {

    module_ = sqrt(std::pow(shelf_pose_y_ - t.transform.translation.y, 2) +
                   std::pow(shelf_pose_x_ - t.transform.translation.x, 2));
    RCLCPP_DEBUG(this->get_logger(), "Modulo  [%.3f]", module_);
    // hallo los vectores unitarios
    //  vectores unitarios a normal
    RCLCPP_DEBUG(this->get_logger(), "Robot position  x [%.3f] y [%.3f]",
                 t.transform.translation.x, t.transform.translation.y);
    RCLCPP_DEBUG(this->get_logger(), "Shelf position  x [%.3f] y [%.3f]",
                 shelf_pose_x_, shelf_pose_y_);

    // Vectores unitarios
    u_y_ = (shelf_pose_y_ - t.transform.translation.y) / module_;
    RCLCPP_DEBUG(this->get_logger(), "u_y_N  [%.3f]", u_y_);

    u_x_ = (shelf_pose_x_ - t.transform.translation.x) / module_;
    RCLCPP_DEBUG(this->get_logger(), "u_x_N  [%.3f]", u_x_);
    // haallo la cantidad de iteraciones par la busqueda
    int cant_ite = static_cast<int>((angle_max_rad_ - angle_min_rad_) /
                                    (2 * resolution_for_find_nav_rad_));
    bool verify_flag = false;
    bool sign = true;
    for (int i; i < cant_ite; i++) {
      if (sign == true) {
        verify_flag = verifyWithCostMap(0.0 + i * resolution_for_find_nav_rad_);
        RCLCPP_DEBUG(this->get_logger(),
                     "Verificando punto de navegacion en direccion  [%.3f]",
                     angle_min_rad_ + i * resolution_for_find_nav_rad_);
        sign = false;
      } else {
        verify_flag = verifyWithCostMap(0.0 - i * resolution_for_find_nav_rad_);
        RCLCPP_DEBUG(this->get_logger(),
                     "Verificando punto de navegacion en direccion  [%.3f]",
                     angle_min_rad_ + i * resolution_for_find_nav_rad_);
        sign = true;
      }

      if (verify_flag) {
        return true;
      }
    }
    return false;
  }

  bool verifyWithCostMap(float direction) {

    // rotar el vector un cierto angulo
    float u_x_R = u_x_ * std::cos(direction) - u_y_ * std::sin(direction);
    float u_y_R = u_x_ * std::sin(direction) + u_y_ * std::cos(direction);

    // hallo los puntos hipotesis te navegacion
    point_nav_.x = shelf_pose_x_ + distance_to_shelf_for_find_pose_ * u_x_R;
    point_nav_.y = shelf_pose_y_ + distance_to_shelf_for_find_pose_ * u_y_R;
    RCLCPP_DEBUG(this->get_logger(), "Punto 1 de navegacion  (%.3f, %.3f)",
                 point_nav_.x, point_nav_.y);
    indexMap map_index = convert_pose_to_map(point_nav_.x, point_nav_.y);

    if (isReachable(costmap_, map_index.x_index, map_index.y_index,
                    cant_casileros_verify_)) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Punto de navecion aceptada (%.3f, %.3f)", point_nav_.x,
                   point_nav_.y);
      return true;
    } else {
      // auto [new_x, new_y] = findNearestReachablePoint(costmap_, goal_x,
      // goal_y); RCLCPP_WARN(this->get_logger(), "Goal not reachable, new goal
      // at (%.3f, %.3f)", new_x, new_y);
      return false;
    }
  }

  bool isReachable(const nav_msgs::msg::OccupancyGrid::SharedPtr &costmap,
                   int x, int y, int neighborhood_size) {
    int width = costmap->info.width;
    int height = costmap->info.height;

    // Verificar la posición central
    int index = y * width + x;
    if (index < 0 || index >= costmap->data.size() ||
        costmap->data[index] >= 50) {
      return false;
    }

    // Verificar la vecindad
    for (int i = 1; i <= neighborhood_size; ++i) {
      // Verificar arriba
      if (y - i >= 0) {
        int index_up = (y - i) * width + x;
        if (index_up < 0 || index_up >= costmap->data.size() ||
            costmap->data[index_up] >= 50) {
          return false;
        }
      }

      // Verificar abajo
      if (y + i < height) {
        int index_down = (y + i) * width + x;
        if (index_down < 0 || index_down >= costmap->data.size() ||
            costmap->data[index_down] >= 50) {
          return false;
        }
      }

      // Verificar izquierda
      if (x - i >= 0) {
        int index_left = y * width + (x - i);
        if (index_left < 0 || index_left >= costmap->data.size() ||
            costmap->data[index_left] >= 50) {
          return false;
        }
      }

      // Verificar derecha
      if (x + i < width) {
        int index_right = y * width + (x + i);
        if (index_right < 0 || index_right >= costmap->data.size() ||
            costmap->data[index_right] >= 50) {
          return false;
        }
      }
    }

    return true;
  }

  indexMap convert_pose_to_map(float goal_x, float goal_y) {
    float resolution_map = costmap_->info.resolution;
    float pose_x_to_map = goal_x - costmap_->info.origin.position.x;
    float pose_y_to_map = goal_y - costmap_->info.origin.position.y;

    int index_x = static_cast<int>(pose_x_to_map / resolution_map);
    int index_y = static_cast<int>(pose_y_to_map / resolution_map);
    indexMap index_map;
    index_map.x_index = index_x;
    index_map.y_index = index_y;
    RCLCPP_DEBUG(this->get_logger(), "Resoluction  [%.3f]", resolution_map);
    RCLCPP_DEBUG(this->get_logger(),
                 "Punto de navegacion index in the map (%i, %i)",
                 index_map.x_index, index_map.y_index);
    return index_map;
  }

  void getTransform(std::string frame_head, std::string frame_child) {

    t.transform.translation.z = std::numeric_limits<double>::max();
    rclcpp::Rate rate(20);
    while (t.transform.translation.z == std::numeric_limits<double>::max() &&
           rclcpp::ok()) {
      try {
        t = tf_buffer_->lookupTransform(frame_head, frame_child,
                                        tf2::TimePointZero);

      } catch (tf2::TransformException &ex) {
        RCLCPP_DEBUG(this->get_logger(), "Llame a lookupTransform");
        RCLCPP_DEBUG(this->get_logger(),
                     "Error al obtener la transformacion de [%s]  a [%s]",
                     frame_head.c_str(), frame_child.c_str());
      }
      RCLCPP_DEBUG(this->get_logger(), "Se obtuvo la transformacion");
      rate.sleep();
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // Instantiate a node.
  rclcpp::Node::SharedPtr node = std::make_shared<FindPointForNav>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}