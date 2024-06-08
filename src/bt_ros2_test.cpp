#include "plugins/BT_localization_init.hpp"
#include "plugins/BT_find_shelf_service_client.hpp"


#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#define DEFAULT_BT_XML "/home/ros/bt_ros2_ws/src/BT_ros2/bt_xml/bt_nav_mememan.xml"

using namespace BT;

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
  
    auto nh = rclcpp::Node::make_shared("bt_ros2_test_node");

    nh->declare_parameter("bt_xml", rclcpp::ParameterValue(std::string("bt_xml.xml")));
    std::string bt_xml;
    nh->get_parameter("bt_xml", bt_xml);

    RCLCPP_INFO(nh->get_logger(), "Loading XML name : %s", bt_xml.c_str());

    // Obtain the directory of the current package
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("rb1_shelf_tools");
    std::string full_bt_xml_path = package_share_directory + "/bt_xml/" + bt_xml;


    RCLCPP_INFO(nh->get_logger(), "Loading XML path : %s", full_bt_xml_path.c_str());
  
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;
  
    factory.registerNodeType<LocalizationInit>("LocalizationInit");
    factory.registerNodeType<FindShelfClient>("FindShelfClient");

    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromFile(full_bt_xml_path);
  
    // Create a logger
    StdCoutLogger logger_cout(tree);
  
    NodeStatus status = NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (rclcpp::ok() && status == NodeStatus::RUNNING) {
        status = tree.tickOnce();
        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  
    return 0;
}
