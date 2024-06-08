#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <vector>
#include <cmath>
using namespace std::chrono_literals;



class TrajectoryGenerate : public rclcpp::Node
{
public:
    TrajectoryGenerate()
        : Node("trajectory_generate")
    {
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("path_attach", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&TrajectoryGenerate::timer_callback, this));

        // Initial and target positions
        start_pose_.x = 2.0;
        start_pose_.y = 2.0;
        start_pose_.theta = 0.0;

        end_pose_.x = 2.5;   // Target position x
        end_pose_.y = 2.5;   // Target position y
        end_pose_.theta = M_PI_2; // Target orientation (90 degrees)


        // Generate the trajectory (spline interpolation)
        generateTrajectory();

        // Publish the trajectory for visualization in RViz
        
    }

private:
    void generateTrajectory()
    {
        double t = 0.0;
        double dt = 0.01;
        while (t <= 1.0)
        {
            double x = (1 - t) * start_pose_.x + t * end_pose_.x;
            double y = (1 - t) * start_pose_.y + t * end_pose_.y;
            double theta = (1 - t) * start_pose_.theta + t * end_pose_.theta;

            trajectory_.emplace_back(x, y, theta);
            t += dt;
        }
        


        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "robot_base_link";

        for (const auto& pose : trajectory_)
        {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.pose.position.x = pose.x;
            pose_stamped.pose.position.y = pose.y;
            tf2::Quaternion q;
            q.setRPY(0, 0, pose.theta);
            pose_stamped.pose.orientation = tf2::toMsg(q);

            path_msg.poses.push_back(pose_stamped);
        }

        path_pub_->publish(path_msg);
    }

    void timer_callback()
    {
        // Publish path.
        RCLCPP_INFO(this->get_logger(), "Publishing path, path size: %d",static_cast<int>(path_msg.poses.size()));
        path_pub_->publish(path_msg);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    nav_msgs::msg::Path path_msg;

    struct Pose2D
    {
        double x;
        double y;
        double theta;

        Pose2D(double x, double y, double theta) : x(x), y(y), theta(theta) {}
    };

    std::vector<Pose2D> trajectory_;
    size_t trajectory_index_ = 0;

    geometry_msgs::msg::Pose2D start_pose_;
    geometry_msgs::msg::Pose2D end_pose_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TrajectoryGenerate>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
