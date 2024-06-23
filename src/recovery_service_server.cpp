#include "rclcpp/rclcpp.hpp"

#include "std_srvs/srv/trigger.hpp"

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


using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

struct indexMap {
  int x_index;
  int y_index;
};

class RecoveryServer : public rclcpp::Node {
public:
  RecoveryServer() : Node("recovery_nav_server") {

    // transfor broadcaster
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // services
    srv_ = create_service<std_srvs::srv::Trigger>("/recovery_nav_server",std::bind(&RecoveryServer::RecoveryNavCallback, this, _1, _2));

    auto sensor_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    // subcrition a mapa de costos
    sub_map_cost_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", sensor_qos,std::bind(&RecoveryServer::costMapCallback, this, _1));

    // inicio en un valor el contenido
    t.transform.translation.z = std::numeric_limits<double>::max();

    arrived_costmap = false;
    // parameteres
   

    RCLCPP_INFO(this->get_logger(),
                "Server de servidor [recovery_nav_server] inicializado ");
  }

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_map_cost_;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap_;

  geometry_msgs::msg::TransformStamped t;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  bool arrived_costmap;

  // laser callback
  void costMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    costmap_ = msg;
    arrived_costmap = true;
    RCLCPP_DEBUG(this->get_logger(), "Costmap arrived");
  }
  // services callback
  void RecoveryNavCallback(
    const std::shared_ptr<std_srvs::srv::Trigger> request,
    const std::shared_ptr<std_srvs::srv::Trigger> response) {
    RCLCPP_DEBUG(this->get_logger(),"-------------------------------------------");

    // pido la transformacion del robot con respecto al map
    getTransform("map", "robot_base_link");
    
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
  rclcpp::Node::SharedPtr node = std::make_shared<RecoveryServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}