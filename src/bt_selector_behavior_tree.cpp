
#include "plugins/BT_approach_shelf_service_client.hpp"
#include "plugins/BT_change_footprint.hpp"
#include "plugins/BT_check_approach.hpp"
#include "plugins/BT_clear_costmap.hpp"
#include "plugins/BT_delay_node.hpp"
#include "plugins/BT_find_shelf_service_client.hpp"
#include "plugins/BT_init_localization_service_client.hpp"
#include "plugins/BT_localization_init.hpp"
#include "plugins/BT_nav_client.hpp"
#include "plugins/BT_publish_transform.hpp"
#include "plugins/BT_publish_transform_back.hpp"
#include "plugins/BT_shelf_handler.hpp"
#include "plugins/BT_turn_robot.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"


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

        sub_bt_select_ = this->create_subscription<std_msgs::msg::String>("bt_selector",10,std::bind(&BehaviorTreeNode::selectorCallback, this, _1));
        
        pub_status_task_ = this->create_publisher<std_msgs::msg::Bool>("status_task",10);


        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        // Obtain the directory of the current package
        package_share_directory_ = ament_index_cpp::get_package_share_directory("rb1_shelf_tools");
        RCLCPP_INFO(this->get_logger(), "Package share directory: %s",
              package_share_directory_.c_str());
    }

private:
    BT::BehaviorTreeFactory factory;
    BT::Tree tree;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_bt_select_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_status_task_;
    std::string bt_select_;
    std::string bt_xml_;
    std::string full_bt_xml_path_;
    std::string package_share_directory_;

    // selector callaback
    void selectorCallback(const std_msgs::msg::String::SharedPtr msg) {
        std_msgs::msg::Bool status_task;
        bt_select_ = msg->data;
        if(bt_select_ == "case1"){
            bt_xml_= "handler_shelf.xml";
            full_bt_xml_path_ = package_share_directory_ + "/bt_xml/" + bt_xml_;
            tree = factory.createTreeFromFile(full_bt_xml_path_);
        }
        if(bt_select_ == "case2"){
            bt_xml_= "find_station_and_init_local.xml";
            full_bt_xml_path_ = package_share_directory_ + "/bt_xml/" + bt_xml_;
            tree = factory.createTreeFromFile(full_bt_xml_path_);
        }
        status_task.data = tickTree();
        pub_status_task_->publish(status_task);
    }


    bool tickTree() {
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
        if(status ==  NodeStatus::SUCCESS){
            return true;
        }
        else{
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