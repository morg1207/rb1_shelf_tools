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


enum class StateFind {
    INIT,
    FALLING,
    RISING
};


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


    this->declare_parameter("limit_intensity_laser_detect",0.0);
    
    limit_intensity_laser_detect = this->get_parameter("limit_intensity_laser_detect").as_double();
    RCLCPP_INFO(this->get_logger(), "Limit intensity laser detect [%.3f] ",limit_intensity_laser_detect);


    // services
    srv_ = create_service<findShelfSrv>(
        "/find_shelf_server",
        std::bind(&findShelfServer::findShelfCallback, this, _1, _2));


    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();


    my_callback_sub = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options;
    options.callback_group = my_callback_sub;
    // subcrition
    sub_laser = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", sensor_qos, std::bind(&findShelfServer::laserCallback, this, _1),options);

    count_legs_ = 0;
    t.transform.translation.z = std::numeric_limits<double>::max();

    RCLCPP_INFO(this->get_logger(), "Server de ervidor [find_shelf_server] inicializado ");

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

  //params
  double limit_intensity_laser_detect; 

  // vector for detection legs
  std::vector<int> index_legs_;
  int count_legs_;


  // laser callback
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    *laser_data_ = *msg;
    RCLCPP_DEBUG(this->get_logger(), "Laser scan arrived");
  }
  // services callback
  void
  findShelfCallback(const std::shared_ptr<findShelfSrv::Request> request,
                          const std::shared_ptr<findShelfSrv::Response> response) {
        RCLCPP_DEBUG(this->get_logger(), "-------------------------------------------");
        bool found_lengs = false; 
        geometry_msgs::msg::Point pos_shelf;

        found_lengs = calculate_index_legs();
        if(found_lengs == false){
          response->success = false;
        }
        else{
          float point_leg[3];
          calculate_point_middle(point_leg);
          pos_shelf.x = point_leg[0];
          pos_shelf.y = point_leg[1];
          pos_shelf.z = point_leg[2];

          response->shelf_position = pos_shelf;
          response->success = true;
        }
  }

  void calculate_point_middle(float *ptr) {
    RCLCPP_DEBUG(this->get_logger(), "Execute calculate point middle");
    
    float angle_increment = laser_data_static_->angle_increment;
    float angle_base = laser_data_static_->angle_min;

    RCLCPP_DEBUG(this->get_logger(), "angle_base  [%.3f]",angle_base);
    RCLCPP_DEBUG(this->get_logger(), "angle_increment  [%.3f]",angle_increment);

    int index_leg_1 = 0.0;
    int index_leg_2 = 0.0 ;

    int prom_index = 0;
    int count = 0 ;
    
    
    for (auto item = index_legs_.begin(); item != index_legs_.end(); item++) {
      if( *item != -1){
        prom_index = prom_index + *item;
        count++;
      }
      else{
          if( item != index_legs_.end()-1 ){
            index_leg_1 = prom_index/count;
            RCLCPP_DEBUG(this->get_logger(), "Prom index leg 1 prom [%d]", index_leg_1);
            prom_index = 0;
            count = 0;
          }
          else{
            index_leg_2 = prom_index/count;
            RCLCPP_DEBUG(this->get_logger(), "Prom index leg 2  prom [%d]", index_leg_2);
            prom_index = 0;
            count = 0;
          }
        
      }
    }
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
    index_legs_.clear();  
    count_legs_ = 0;       

    StateFind state_find;

    state_find = StateFind::INIT;

    *laser_data_static_ = *laser_data_;
    std::vector<float> intensities = laser_data_static_->intensities;

    RCLCPP_DEBUG(this->get_logger(), "Size [%ld] ", intensities.size());

    for (auto item = intensities.begin(); item != intensities.end(); item++) {

      switch (state_find){
        case StateFind::INIT:
            if (*item > limit_intensity_laser_detect){
                state_find = StateFind::RISING;
                index_legs_.push_back(item - intensities.begin());
                RCLCPP_DEBUG(this->get_logger(),"Leg number [%d] -- Index añadido [%ld] -- Intensidad [%.2f]", count_legs_ ,item - intensities.begin(),*item);
            }
            break;
        case StateFind::RISING:
            if (*item > limit_intensity_laser_detect){
                 index_legs_.push_back(item - intensities.begin());
                RCLCPP_DEBUG(this->get_logger(),"Leg number [%d] -- Index añadido [%ld] -- Intensidad [%.2f]", count_legs_ ,item - intensities.begin(),*item);
            }
            else{
                count_legs_ ++;
                RCLCPP_DEBUG(this->get_logger(), "Found [%d legs", count_legs_);
                state_find = StateFind::FALLING;
                index_legs_.push_back(-1);
            }
            break;
        case StateFind::FALLING:
            if (*item > limit_intensity_laser_detect){
                index_legs_.push_back(item - intensities.begin());
                RCLCPP_DEBUG(this->get_logger(),"Leg number [%d] -- Index añadido [%ld] -- Intensidad [%.2f]", count_legs_ ,item - intensities.begin(),*item);
                state_find = StateFind::RISING;
            }
            break;

      }
    }
    if (count_legs_ == 2) {
      RCLCPP_DEBUG(this->get_logger(), "found[%d] legs ", count_legs_);
      return true;
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Failed found [%d] legs ", count_legs_);
      return false;
    }
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