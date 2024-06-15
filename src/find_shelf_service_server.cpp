#include "rclcpp/rclcpp.hpp"

#include "rb1_shelf_msgs/srv/find_shelf.hpp"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

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
using findShelfSrv = rb1_shelf_msgs::srv::FindShelf;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

enum class StateFind { INIT, FALLING, RISING };

class findShelfServer : public rclcpp::Node {
public:
  findShelfServer() : Node("find_shelf_server") {

    // transfor broadcaster
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // data laser_scan
    laser_data_ = std::make_shared<sensor_msgs::msg::LaserScan>();
    // data laser_scan
    laser_data_static_ = std::make_shared<sensor_msgs::msg::LaserScan>();

    // services
    srv_ = create_service<findShelfSrv>(
        "/find_shelf_server",
        std::bind(&findShelfServer::findShelfCallback, this, _1, _2));

    auto sensor_qos =
        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    my_callback_sub =
        create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = my_callback_sub;
    // subcrition
    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", sensor_qos,
        std::bind(&findShelfServer::laserCallback, this, _1), options);

    t.transform.translation.z = std::numeric_limits<double>::max();

    // Parametros
    this->declare_parameter("limit_intensity_laser_detect", 0.0);
    limit_intensity_laser_detect_ =
        this->get_parameter("limit_intensity_laser_detect").as_double();
    RCLCPP_INFO(this->get_logger(), "Limit intensity laser detect [%.3f] ",
                limit_intensity_laser_detect_);
    // parametros shelf
    this->declare_parameter("limit_min_detection_distance_legs_shelf", 0.55);
    limit_min_detection_distance_legs_shelf_ =
        this->get_parameter("limit_min_detection_distance_legs_shelf")
            .as_double();
    RCLCPP_INFO(this->get_logger(),
                "limit min detection distance legs shelf [%.3f] ",
                limit_min_detection_distance_legs_shelf_);

    this->declare_parameter("limit_max_detection_distance_legs_shelf", 0.70);
    limit_max_detection_distance_legs_shelf_ =
        this->get_parameter("limit_max_detection_distance_legs_shelf")
            .as_double();
    RCLCPP_INFO(this->get_logger(),
                "limit max detection distance legs shelf [%.3f] ",
                limit_max_detection_distance_legs_shelf_);
    // parametros docker station
    this->declare_parameter("limit_min_detection_distance_legs_charge_station",
                            0.25);
    limit_min_detection_distance_legs_charge_station_ =
        this->get_parameter("limit_min_detection_distance_legs_charge_station")
            .as_double();
    RCLCPP_INFO(this->get_logger(),
                "limit min detection distance legs charge station [%.3f] ",
                limit_min_detection_distance_legs_charge_station_);

    this->declare_parameter("limit_max_detection_distance_legs_charge_station",
                            0.35);
    limit_max_detection_distance_legs_charge_station_ =
        this->get_parameter("limit_max_detection_distance_legs_charge_station")
            .as_double();
    RCLCPP_INFO(this->get_logger(),
                "limit max detection distance legs shelf [%.3f] ",
                limit_max_detection_distance_legs_charge_station_);

    RCLCPP_INFO(this->get_logger(),
                "Server de ervidor [find_shelf_server] inicializado ");
  }

private:
  rclcpp::Service<findShelfSrv>::SharedPtr srv_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laser;
  rclcpp::CallbackGroup::SharedPtr my_callback_sub;

  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_data_;
  std::shared_ptr<sensor_msgs::msg::LaserScan> laser_data_static_;

  geometry_msgs::msg::TransformStamped t;

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  // params
  double limit_intensity_laser_detect_;
  double limit_min_detection_distance_legs_shelf_;
  double limit_max_detection_distance_legs_shelf_;
  double limit_min_detection_distance_legs_charge_station_;
  double limit_max_detection_distance_legs_charge_station_;

  // vector for detection legs
  std::vector<int> index_legs_;
  std::vector<int> index_shelf_;
  std::vector<int> index_legs_middle_point_;

  // laser callback
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    *laser_data_ = *msg;
    // RCLCPP_DEBUG(this->get_logger(), "Laser scan arrived");
  }
  // services callback
  void
  findShelfCallback(const std::shared_ptr<findShelfSrv::Request> request,
                    const std::shared_ptr<findShelfSrv::Response> response) {
    RCLCPP_DEBUG(this->get_logger(),
                 "-------------------------------------------");
    RCLCPP_DEBUG(this->get_logger(), "Find server [%s]",
                 request->object_find.c_str());

    bool found_legs = false;
    float count_shelf;
    geometry_msgs::msg::Point pos_shelf;
    // Reinicio las matrices de index
    index_shelf_.clear();
    index_legs_.clear();
    index_legs_middle_point_.clear();

    found_legs = calculate_index_legs();
    if (found_legs != true) {
      response->success = false;
    } else {
      count_shelf = verify_legs_shelf_or_station_charge(request->object_find);
      if (count_shelf != 1) {
        response->success = false;
      } else {
        float point_leg[3];
        calculate_point_middle(point_leg);
        pos_shelf.x = point_leg[0];
        pos_shelf.y = point_leg[1];
        pos_shelf.z = point_leg[2];
        response->shelf_position = pos_shelf;
        response->success = true;
      }
    }
    RCLCPP_DEBUG(this->get_logger(),
                 "-------------------------------------------");
  }

  void calculate_point_middle(float *ptr) {
    RCLCPP_DEBUG(this->get_logger(), "Execute calculate point middle");

    float angle_increment = laser_data_static_->angle_increment;
    float angle_base = laser_data_static_->angle_min;

    RCLCPP_DEBUG(this->get_logger(), "angle_base  [%.3f]", angle_base);
    RCLCPP_DEBUG(this->get_logger(), "angle_increment  [%.3f]",
                 angle_increment);

    int index_leg_1 = index_shelf_[0];
    int index_leg_2 = index_shelf_[1];

    float theta_a_leg = (index_leg_1 * angle_increment + angle_base);
    float theta_b_leg = (index_leg_2 * angle_increment + angle_base);
    RCLCPP_DEBUG(this->get_logger(), "theta_a_leg [%.3f]", theta_a_leg);
    RCLCPP_DEBUG(this->get_logger(), "theta_b_leg [%.3f]", theta_b_leg);
    float d1 = laser_data_static_->ranges[index_leg_1];
    float d2 = laser_data_static_->ranges[index_leg_2];
    RCLCPP_DEBUG(this->get_logger(), "d1 [%.3f]", d1);
    RCLCPP_DEBUG(this->get_logger(), "d2 [%.3f]", d2);
    float dt_alfa = d1 * std::cos(theta_a_leg) + d2 * std::cos(theta_b_leg);
    float dt_beta = d1 * std::sin(theta_a_leg) + d2 * std::sin(theta_b_leg);
    RCLCPP_DEBUG(this->get_logger(), "dt_alfa [%.3f]", dt_alfa);
    RCLCPP_DEBUG(this->get_logger(), "dt_beta [%.3f]", dt_beta);
    float dt = std::sqrt(std::pow(dt_alfa, 2) + std::pow(dt_beta, 2)) / 2;
    float theta_total = std::atan2(dt_beta, dt_alfa);
    RCLCPP_DEBUG(this->get_logger(), "dt [%.3f]", dt);
    RCLCPP_DEBUG(this->get_logger(), "theta_total [%.3f]", theta_total);
    float c = std::sqrt(std::pow(d2, 2) + std::pow(dt, 2) -
                        2 * d2 * dt * std::cos(abs(theta_total - theta_b_leg)));
    RCLCPP_DEBUG(this->get_logger(), "c [%.3f]", c);
    float alfa_theta = abs(std::acos(
        (std::pow(c, 2) + std::pow(dt, 2) - std::pow(d2, 2)) / (2 * c * dt)));
    RCLCPP_DEBUG(this->get_logger(), "alfa_theta [%.3f]", alfa_theta);
    float theta_final = PI / 2 + theta_total - alfa_theta;
    RCLCPP_DEBUG(this->get_logger(), "theta_final [%.3f]", theta_final);
    ptr[0] = dt;
    ptr[1] = theta_total;
    ptr[2] = theta_final;
  }

  bool calculate_index_legs() {

    int count_legs = 0;

    StateFind state_find;

    state_find = StateFind::INIT;

    *laser_data_static_ = *laser_data_;
    std::vector<float> intensities = laser_data_static_->intensities;

    RCLCPP_DEBUG(this->get_logger(), "Size [%ld] ", intensities.size());

    for (auto item = intensities.begin(); item != intensities.end(); item++) {

      switch (state_find) {
      case StateFind::INIT:
        if (*item > limit_intensity_laser_detect_) {
          state_find = StateFind::RISING;
          index_legs_.push_back(item - intensities.begin());
          RCLCPP_DEBUG(
              this->get_logger(),
              "Leg number [%d] -- Index añadido [%ld] -- Intensidad [%.2f]",
              count_legs, item - intensities.begin(), *item);
        }
        break;
      case StateFind::RISING:
        if (*item > limit_intensity_laser_detect_) {
          index_legs_.push_back(item - intensities.begin());
          RCLCPP_DEBUG(
              this->get_logger(),
              "Leg number [%d] -- Index añadido [%ld] -- Intensidad [%.2f]",
              count_legs, item - intensities.begin(), *item);
        } else {
          count_legs++;
          RCLCPP_DEBUG(this->get_logger(), "Found [%d legs", count_legs);
          state_find = StateFind::FALLING;
          index_legs_.push_back(-1);
        }
        break;
      case StateFind::FALLING:
        if (*item > limit_intensity_laser_detect_) {
          index_legs_.push_back(item - intensities.begin());
          RCLCPP_DEBUG(
              this->get_logger(),
              "Leg number [%d] -- Index añadido [%ld] -- Intensidad [%.2f]",
              count_legs, item - intensities.begin(), *item);
          state_find = StateFind::RISING;
        }
        break;
      }
    }
    if (count_legs > 1) {
      RCLCPP_DEBUG(this->get_logger(), "found[%d] legs ", count_legs);
      return true;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Failed found [%d] legs ", count_legs);
      return false;
    }
  }
  int verify_legs_shelf_or_station_charge(std::string object_find) {
    // Borro el array del index shelf

    int numb_leg = 0;
    int prom_index = 0;
    int count = 0;
    int count_shelf = 0;
    // Hallo el punto medio de cada pata
    for (auto item = index_legs_.begin(); item != index_legs_.end(); item++) {
      if (*item != -1) {
        prom_index = prom_index + *item;
        count++;
      } else {
        if (item != index_legs_.end() - 1) {
          index_legs_middle_point_.push_back(prom_index / count);
          RCLCPP_DEBUG(this->get_logger(), "Prom index leg [%d] prom [%d]",
                       numb_leg, prom_index / count);
          numb_leg++;
          prom_index = 0;
          count = 0;
        } else {
          index_legs_middle_point_.push_back(prom_index / count);
          RCLCPP_DEBUG(this->get_logger(), "Prom index leg [%d]  prom [%d]",
                       numb_leg, prom_index / count);
          prom_index = 0;
          count = 0;
        }
      }
    }
    float angle_increment = laser_data_static_->angle_increment;
    float angle_base = laser_data_static_->angle_min;
    for (auto item_i = index_legs_middle_point_.begin();
         item_i != index_legs_middle_point_.end() - 1; item_i++) {

      for (auto item_j = index_legs_middle_point_.begin() + 1;
           item_j != index_legs_middle_point_.end(); item_j++) {

        if (item_j > item_i) {
          RCLCPP_DEBUG(this->get_logger(),
                       "Verificación index [%d] con index [%d] ", *item_i,
                       *item_j);
          float theta_a_leg = (*item_i * angle_increment + angle_base);
          float theta_b_leg = (*item_j * angle_increment + angle_base);
          float d1 = laser_data_static_->ranges[*item_i];
          float d2 = laser_data_static_->ranges[*item_j];
          float theta_final = theta_a_leg - theta_b_leg;
          float distance_between_legs =
              std::sqrt(std::pow(d1, 2) + std::pow(d2, 2) -
                        2 * d1 * d2 * std::cos(theta_final));
          RCLCPP_DEBUG(
              this->get_logger(),
              "Distancia entre patas [%.3f] con index [%d] y con index [%d] ",
              distance_between_legs, *item_i, *item_j);

          // Verifico la distancia
          if (object_find == "shelf") {
            if (distance_between_legs >
                    limit_min_detection_distance_legs_shelf_ &&
                distance_between_legs <
                    limit_max_detection_distance_legs_shelf_) {
              index_shelf_.push_back(*item_i);
              index_shelf_.push_back(*item_j);
              RCLCPP_DEBUG(
                  this->get_logger(),
                  "Shelf numero   [%d] con index [%d] y con index [%d] ",
                  count_shelf, *item_i, *item_j);
              RCLCPP_DEBUG(this->get_logger(),
                           "**********************************************");
              count_shelf++;
            }
          }
          if (object_find == "station") {
            if (distance_between_legs >
                    limit_min_detection_distance_legs_charge_station_ &&
                distance_between_legs <
                    limit_max_detection_distance_legs_charge_station_) {
              index_shelf_.push_back(*item_i);
              index_shelf_.push_back(*item_j);
              RCLCPP_DEBUG(
                  this->get_logger(),
                  "Estacion de carga numero   [%d] con index [%d] y con "
                  "index [%d] ",
                  count_shelf, *item_i, *item_j);
              RCLCPP_DEBUG(this->get_logger(),
                           "**********************************************");
              count_shelf++;
            }
          }
        }
      }
    }
    return count_shelf;
  }
};

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