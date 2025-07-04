#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class ClockPoseIssuerNode : public rclcpp::Node
{
public:
    ClockPoseIssuerNode() : Node("clock_pose_issuer")
    {
        // Parameters
        this->declare_parameter("timer_period", 6.0);
        timer_period_ = this->get_parameter("timer_period").as_double();
        
        // Publisher
        clock_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/clock_pose", 10);

        // Timer
        timer_ = this->create_wall_timer(
                std::chrono::duration<double>(timer_period_), 
                std::bind(&ClockPoseIssuerNode::publishClockPose, this));
        RCLCPP_INFO(this->get_logger(), "Clock Pose Node has started.");
    }

private:
    void publishClockPose()
    {
        // Extract the current time -- minute
        auto now = std::chrono::system_clock::now();
        auto time_t_now = std::chrono::system_clock::to_time_t(now);
        auto local_time = *std::localtime(&time_t_now);
        double current_minute = static_cast<double>(local_time.tm_min); // Extract the current minute

        // Calculate the angle of the minute hand
        double angle = (1 - current_minute / 60.0) * 2.0 * M_PI; // [radians]
        
        // Calculate position on the unit circle
        double x = std::cos(angle); 
        double y = std::sin(angle); 

        // Create PoseStamped msg
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "map";  // Use "map" as a reference frame
        
        // Set position
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = 0.0;

        // Set orientation
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, angle);
        msg.pose.orientation.x = quat.x();
        msg.pose.orientation.y = quat.y();
        msg.pose.orientation.z = quat.z();
        msg.pose.orientation.w = quat.w();

        // Publish clock pose
        clock_pose_pub_->publish(msg);

        // Log for debugging
        RCLCPP_INFO(this->get_logger(), "Published clock pose at minute %.2f: [x=%.2f, y=%.2f, angle=%.2f radians]",
                                                                current_minute, x, y, angle);
    }
    // Publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr clock_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Parameters
    double timer_period_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClockPoseIssuerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}