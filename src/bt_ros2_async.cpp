
#include "geometry_msgs/msg/pose.hpp"
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
#include "rcl/publisher.h"

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#define DEFAULT_BT_XML                                                         \
  "/home/ros/bt_ros2_ws/src/BT_ros2/bt_xml/bt_nav_mememan.xml"

using namespace BT;

int main(int argc, char **argv) {

  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  auto nh = rclcpp::Node::make_shared("bt_ros2_async_node");

  nh->declare_parameter("bt_xml",
                        rclcpp::ParameterValue(std::string("bt_xml.xml")));
  std::string bt_xml;
  nh->get_parameter("bt_xml", bt_xml);

  RCLCPP_INFO(nh->get_logger(), "Loading XML name : %s", bt_xml.c_str());

  // Obtain the directory of the current package
  std::string package_share_directory =
      ament_index_cpp::get_package_share_directory("rb1_shelf_tools");
  std::string full_bt_xml_path = package_share_directory + "/bt_xml/" + bt_xml;

  RCLCPP_INFO(nh->get_logger(), "Loading XML path : %s",
              full_bt_xml_path.c_str());
  BT::BehaviorTreeFactory factory;

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

  auto tree = factory.createTreeFromFile(full_bt_xml_path);
  // Create a logger
  StdCoutLogger logger_cout(tree);
  // Here, instead of tree.tickWhileRunning(),
  // we prefer our own loop.
  std::cout << "--- ticking\n";
  auto status = tree.tickOnce();
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

  return 0;
}