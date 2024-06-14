#include "rclcpp/rclcpp.hpp"

#include "rb1_shelf_msgs/srv/init_localization.hpp"


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
using initLocsrc = rb1_shelf_msgs::srv::InitLocalization
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;




class InitLocalizationServer : public rclcpp::Node {
public:
  InitLocalizationServer() : Node("init_localization_server") {

    // transfor broadcaster
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // services
    srv_ = create_service<initLocsrc>(
        "/find_shelf_server",
        std::bind(&InitLocalizationServer::initLocalizationCallback, this, _1, _2));


    t.transform.translation.z = std::numeric_limits<double>::max();

    // Parametros 
    this->declare_parameter("pose_map_to_station_charge_x",0.0);
    pose_map_to_station_charge_x_ = this->get_parameter("pose_map_to_station_charge_x").as_double();
    RCLCPP_INFO(this->get_logger(), "Pose map to station charge x [%.3f] ",pose_map_to_station_charge_x_);

    // Parametros 
    this->declare_parameter("pose_map_to_station_charge_Y",0.0);
    pose_map_to_station_charge_x_ = this->get_parameter("pose_map_to_station_charge_x").as_double();
    RCLCPP_INFO(this->get_logger(), "Pose map to station charge x [%.3f] ",pose_map_to_station_charge_x_);
    
    // Parametros 
    this->declare_parameter("pose_map_to_station_charge_x",0.0);
    pose_map_to_station_charge_x_ = this->get_parameter("pose_map_to_station_charge_x").as_double();
    RCLCPP_INFO(this->get_logger(), "Pose map to station charge x [%.3f] ",pose_map_to_station_charge_x_);

  
    pose_map_to_station_charge_x: 
    pose_map_to_station_charge_y: 
    pose_map_to_station_charge_yaw: 


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
    //RCLCPP_DEBUG(this->get_logger(), "Laser scan arrived");
  }
  // services callback
  void
  findShelfCallback(const std::shared_ptr<findShelfSrv::Request> request,
                          const std::shared_ptr<findShelfSrv::Response> response) {
        RCLCPP_DEBUG(this->get_logger(), "-------------------------------------------");
        bool found_legs = false; 
        float count_shelf;
        geometry_msgs::msg::Point pos_shelf;
        // Reinicio las matrices de index
        index_shelf_.clear(); 
        index_legs_.clear();  
        index_legs_middle_point_.clear();

        found_legs = calculate_index_legs();
        if(found_legs != true){
          response->success = false;
        }
        else{
          count_shelf = verify_legs_shelf_or_station_charge(request->object_find);
          if (count_shelf != 1){
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
        RCLCPP_DEBUG(this->get_logger(), "-------------------------------------------");
        
  }

  void calculate_point_middle(float *ptr) {

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