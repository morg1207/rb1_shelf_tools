
#include "plugins/BT_approach_shelf_service_client.hpp"
#include "plugins/BT_cancel_nav.hpp"
#include "plugins/BT_change_footprint.hpp"
#include "plugins/BT_check_approach.hpp"
#include "plugins/BT_check_nav.hpp"
#include "plugins/BT_clear_costmap.hpp"
#include "plugins/BT_decorator_force_success.hpp"
#include "plugins/BT_delay_node.hpp"
#include "plugins/BT_find_nav_points_service_client.hpp"
#include "plugins/BT_find_shelf_deep_service_client.hpp"
#include "plugins/BT_find_shelf_service_client.hpp"
#include "plugins/BT_init_localization_service_client.hpp"
#include "plugins/BT_localization_init.hpp"
#include "plugins/BT_nav2_dischargePose.hpp"
#include "plugins/BT_nav_client.hpp"
#include "plugins/BT_publish_state_robot.hpp"
#include "plugins/BT_publish_transform.hpp"
#include "plugins/BT_publish_transform_back.hpp"
#include "plugins/BT_shelf_handler.hpp"
#include "plugins/BT_turn_robot.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#define DEFAULT_BT_XML                                                         \
  "/home/ros/bt_ros2_ws/src/BT_ros2/bt_xml/bt_nav_mememan.xml"

using namespace BT;

class BehaviorTreeNode : public rclcpp::Node {

public:
  BehaviorTreeNode() : Node("behavior_tree_node") {

    factory.registerNodeType<PublishTransform>("PublishTransform");
    factory.registerNodeType<PublishTransformBack>("PublishTransformBack");
    factory.registerNodeType<FindShelfClient>("FindShelfClient");
    factory.registerNodeType<Nav2Client>("Nav2Client");
    factory.registerNodeType<TurnRobot>("TurnRobot");
    factory.registerNodeType<LocalizationInit>("LocalizationInit");
    factory.registerNodeType<ClearCostmap>("ClearCostmap");
    factory.registerNodeType<ApproachShelfClient>("ApproachShelfClient");
    factory.registerNodeType<CheckApproach>("CheckApproach");
    factory.registerNodeType<ShelfHandler>("ShelfHandler");
    factory.registerNodeType<DelayNodeBT>("DelayNodeBT");
    factory.registerNodeType<ChangeFootprint>("ChangeFootprint");
    factory.registerNodeType<InitLocalizationClient>("InitLocalizationClient");
    factory.registerNodeType<CheckNavGoal>("CheckNavGoal");
    factory.registerNodeType<ForceSuccess>("ForceSuccessDeco");
    factory.registerNodeType<Nav2DischargePose>("Nav2DischargePose");
    factory.registerNodeType<FindShelDeepfClient>("FindShelDeepfClient");
    factory.registerNodeType<FindNavPointsClient>("FindNavPointsClient");
    factory.registerNodeType<CancelNav>("CancelNav");
    factory.registerNodeType<PublishStateRobot>("PublishStateRobot");

    sub_bt_select_ = this->create_subscription<std_msgs::msg::String>(
        "bt_selector", 10,
        std::bind(&BehaviorTreeNode::selectorCallback, this, _1));

    pub_status_task_ =
        this->create_publisher<std_msgs::msg::String>("bt_status", 10);

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    // Obtain the directory of the current package
    package_share_directory_ =
        ament_index_cpp::get_package_share_directory("rb1_shelf_tools");
    RCLCPP_INFO(this->get_logger(), "Package share directory: %s",
                package_share_directory_.c_str());
  }

private:
  BT::BehaviorTreeFactory factory;
  BT::Tree tree;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_bt_select_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_status_task_;
  std::string bt_select_;
  std::string bt_xml_;
  std::string full_bt_xml_path_;
  std::string package_share_directory_;

  // selector callaback
  void selectorCallback(const std_msgs::msg::String::SharedPtr msg) {
    bool status_task;
    std_msgs::msg::String status_task_msg;
    bt_select_ = msg->data;
    if (bt_select_ == "find_station_and_init_local") {
      full_bt_xml_path_ =
          package_share_directory_ + "/bt_xml/" + bt_select_ + ".xml";
      tree = factory.createTreeFromFile(full_bt_xml_path_);
      RCLCPP_INFO(this->get_logger(),
                  "Arbol de comportamiento selecsionado: %s",
                  bt_select_.c_str());
    }
    if (bt_select_ == "find_shelf_and_publish") {
      full_bt_xml_path_ =
          package_share_directory_ + "/bt_xml/" + bt_select_ + ".xml";
      tree = factory.createTreeFromFile(full_bt_xml_path_);
      RCLCPP_INFO(this->get_logger(),
                  "Arbol de comportamiento selecsionado: %s",
                  bt_select_.c_str());
    }
    if (bt_select_ == "approach_and_pick_shelf") {
      full_bt_xml_path_ =
          package_share_directory_ + "/bt_xml/" + bt_select_ + ".xml";
      tree = factory.createTreeFromFile(full_bt_xml_path_);
      RCLCPP_INFO(this->get_logger(),
                  "Arbol de comportamiento selecsionado: %s",
                  bt_select_.c_str());
    }
    if (bt_select_ == "carry_and_discharge_shelf") {
      full_bt_xml_path_ =
          package_share_directory_ + "/bt_xml/" + bt_select_ + ".xml";
      tree = factory.createTreeFromFile(full_bt_xml_path_);
      RCLCPP_INFO(this->get_logger(),
                  "Arbol de comportamiento selecsionado: %s",
                  bt_select_.c_str());
    }
    if (bt_select_ == "behaviortree_entire") {
      full_bt_xml_path_ =
          package_share_directory_ + "/bt_xml/" + bt_select_ + ".xml";
      tree = factory.createTreeFromFile(full_bt_xml_path_);
      RCLCPP_INFO(this->get_logger(),
                  "Arbol de comportamiento selecsionado: %s",
                  bt_select_.c_str());
    }

    status_task = tickTree();
    if (status_task) {

      status_task_msg.data = bt_select_ + "/done";
    } else {
      status_task_msg.data = bt_select_ + "/fail";
    }
    pub_status_task_->publish(status_task_msg);
  }

  bool tickTree() {
    auto status = tree.tickOnce();
    // Create a logger
    StdCoutLogger logger_cout(tree);
    std::cout << "--- status: " << toStr(status) << "\n\n";

    while (status == NodeStatus::RUNNING && rclcpp::ok()) {
      // Sleep to avoid busy loops.
      // do NOT use other sleep functions!
      // Small sleep time is OK, here we use a large one only to
      // have less messages on the console.
      tree.sleep(std::chrono::milliseconds(50));

      std::cout << "--- ticking\n";
      status = tree.tickOnce();
      std::cout << "--- status: " << toStr(status) << "\n\n";
    }
    if (status == NodeStatus::SUCCESS) {
      return true;
    } else {
      return false;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<BehaviorTreeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}