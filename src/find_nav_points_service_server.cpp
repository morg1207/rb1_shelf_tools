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
    RCLCPP_INFO(this->get_logger(),
                "Server de ervidor [find_nav_poses] inicializado ");
  }

private:
  rclcpp::Service<findNavPoses>::SharedPtr srv_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_cost_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;

  geometry_msgs::msg::TransformStamped t;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // parameters
  double distance_to_shelf_for_find_pose_;
  float robot_pose_x_;
  float robot_pose_y_;
  float shelf_pose_x_;
  float shelf_pose_y_;

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

    // pido la transformacion del robot con respecto al map
    getTransform("map", "robot_base_link");

    // Hallo el vector unitario de la posicion del robot con respecto al shelf

    RCLCPP_INFO(this->get_logger(), "passs calculate points");
    response->nav_position = calculate_points();
    RCLCPP_INFO(this->get_logger(), "passs nav_positions");
    response->success = true;
  }
  // funcion para encontrar la transformacion del robot con respecto al shelf
  geometry_msgs::msg::Point calculate_points() {

    float modulo = sqrt(std::pow(shelf_pose_y_ - t.transform.translation.y, 2) +
                        std::pow(shelf_pose_x_ - t.transform.translation.x, 2));
    RCLCPP_INFO(this->get_logger(), "Modulo  [%.3f]", modulo);
    // hallo los vectores unitarios
    //  vectores unitarios a normal
    RCLCPP_INFO(this->get_logger(), "Robot position  x [%.3f] y [%.3f]",
                t.transform.translation.x, t.transform.translation.y);
    RCLCPP_INFO(this->get_logger(), "Shelf position  x [%.3f] y [%.3f]",
                shelf_pose_x_, shelf_pose_y_);

    float u_x_N = (shelf_pose_y_ - t.transform.translation.y) / modulo;
    RCLCPP_INFO(this->get_logger(), "u_y_N  [%.3f]", u_x_N);

    float u_y_N = -(shelf_pose_x_ - t.transform.translation.x) / modulo;
    RCLCPP_INFO(this->get_logger(), "u_x_N  [%.3f]", u_y_N);

    // hallo los puntos inciiales de navegacion
    // Punto 1
    float point_1_x = shelf_pose_x_ + distance_to_shelf_for_find_pose_ * u_x_N;
    float point_1_y = shelf_pose_y_ + distance_to_shelf_for_find_pose_ * u_y_N;
    RCLCPP_INFO(this->get_logger(), "Punto 1 de navegacion  (%.3f, %.3f)",
                point_1_x, point_1_y);

    // Punto 2
    float point_2_x = shelf_pose_x_ - distance_to_shelf_for_find_pose_ * u_x_N;
    float point_2_y = shelf_pose_y_ - distance_to_shelf_for_find_pose_ * u_y_N;
    RCLCPP_INFO(this->get_logger(), "Punto 2 de navegacion  (%.3f, %.3f)",
                point_2_x, point_2_y);

    indexMap map_index_1 = convert_pose_to_map(point_1_x, point_1_y);
    indexMap map_index_2 = convert_pose_to_map(point_2_x, point_2_y);

    verifyWithCostMap(map_index_1.x_index, map_index_1.y_index);

    geometry_msgs::msg::Point point_nav;
    point_nav.x = point_1_x;
    point_nav.y = point_1_y;

    return point_nav;
  }

  void verifyWithCostMap(float goal_x, float goal_y) {

    if (isReachable(costmap_, goal_x, goal_y)) {
      RCLCPP_INFO(this->get_logger(), "Goal is reachable at (%.3f, %.3f)",
                  goal_x, goal_y);
    } else {
      // auto [new_x, new_y] = findNearestReachablePoint(costmap_, goal_x,
      // goal_y); RCLCPP_WARN(this->get_logger(), "Goal not reachable, new goal
      // at (%.3f, %.3f)", new_x, new_y);
    }
  }

  bool isReachable(const nav_msgs::msg::OccupancyGrid::SharedPtr &costmap,
                   int x, int y) {
    int index = y * costmap->info.width + x;
    if (index < 0 || index >= costmap->data.size()) {
      return false;
    }
    return costmap->data[index] < 50;
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
    RCLCPP_INFO(this->get_logger(), "Resoluction  [%.3f]", resolution_map);
    RCLCPP_INFO(this->get_logger(),
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
        RCLCPP_ERROR(this->get_logger(), "Llame a lookupTransform");
        RCLCPP_ERROR(this->get_logger(),
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