#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <cmath>
#include <string>
#include <iostream>
#include <queue>

using namespace std::chrono_literals;

class GuiCliPoseNode : public rclcpp::Node
{
public:
    GuiCliPoseNode() : Node("gui_cli_pose")
    {
        gui_cli_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/gui_cli_pose", 10);
        timer_ = this->create_wall_timer(
                1s, 
                std::bind(&GuiCliPoseNode::publishGuiCliPose, this));
        RCLCPP_INFO(this->get_logger(), "Gui/Cli Pose Node has started.");
    }

private:
    void publishGuiCliPose()
    {
        std::string input;
        std::cout << "Enter minute hand from [1, 60] (or press <space> and Enter to use clock pose): ";

        double current_minute;
        std::getline(std::cin, input);

        if (input == " ")
        {
            // Extract the current time -- minute
            auto now = std::chrono::system_clock::now();
            auto time_t_now = std::chrono::system_clock::to_time_t(now);
            auto local_time = *std::localtime(&time_t_now);
            current_minute = static_cast<double>(local_time.tm_min); // Extract the current minute
        }
        else
        {
            try 
            {
                current_minute = std::stod(input);
            }
            catch (const std::exception& e)
            {
                RCLCPP_WARN(this->get_logger(), "Caught error in: %s", e.what());
            }
        }


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
        gui_cli_pose_pub_->publish(msg);

        // Log for debugging
        RCLCPP_INFO(this->get_logger(), "Published gui pose for input minute %.2f: [x=%.2f, y=%.2f, angle=%.2f radians]",
                                                                current_minute, x, y, angle);
    }

    // Publisher and timer
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr gui_cli_pose_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // queue
    std::queue<double> next_pose_in_line_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GuiCliPoseNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}