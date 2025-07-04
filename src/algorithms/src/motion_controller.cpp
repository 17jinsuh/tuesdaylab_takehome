// Libraries
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>

// Msg and Srv Interfaces
#include "interfaces/srv/get_target_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

using namespace std::placeholders;
using namespace std::chrono_literals;

class MotionControllerNode : public rclcpp::Node 
{
public:
    MotionControllerNode() : Node("motion_controller"), update_prior_current_pose_(true), updated_target_available_(false)
    {
        // Parameters
        this->declare_parameter("kp_linear", 1.0);
        this->declare_parameter("kp_angular", 1.0);
        this->declare_parameter("distance_threshold", 0.5);
        this->declare_parameter("angle_threshold", 0.5);
        this->declare_parameter("control_loop_timer_period", 1.0);
        kp_linear_ = this->get_parameter("kp_linear").as_double();
        kp_angular_ = this->get_parameter("kp_angular").as_double();
        distance_threshold_ = this->get_parameter("distance_threshold").as_double();
        angle_threshold_ = this->get_parameter("angle_threshold").as_double();
        control_loop_timer_period_ = this->get_parameter("control_loop_timer_period").as_double();

        // Subscribers
        target_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_pose", 10, std::bind(&MotionControllerNode::callbackTargetPose, this, _1));
        current_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, std::bind(&MotionControllerNode::callbackCurrentTurtleSimPose, this, _1));

        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        
        // Server Client
        get_target_pose_client_ = this->create_client<interfaces::srv::GetTargetPose>("get_target_pose");

        // Timer for control loop
        control_loop_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(control_loop_timer_period_), std::bind(&MotionControllerNode::controlLoop, this));
        

        RCLCPP_INFO(this->get_logger(), "Motion Controller node has started.");
    }

private:
    void callGetTargetPose()
    {
        while (!get_target_pose_client_->wait_for_service(1s))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }
        auto request = std::make_shared<interfaces::srv::GetTargetPose::Request>();
        request->need_new_target = true;

        get_target_pose_client_->async_send_request(
            request, std::bind(&MotionControllerNode::callbackCallGetTargetPose, this, _1));
    }

    void callbackCallGetTargetPose(rclcpp::Client<interfaces::srv::GetTargetPose>::SharedFuture future)
    {
        auto response = future.get();

        updated_target_available_ = response->updated_target;
        if (updated_target_available_)
        {
            target_pose_.pose.position.x +=  prior_current_pose_.pose.position.x;
            target_pose_.pose.position.y +=  prior_current_pose_.pose.position.y;
            target_pose_.pose.position.z +=  prior_current_pose_.pose.position.z;
        }
        else
        {
            target_pose_ = prior_current_pose_;
        }
        // Log for debugging
        RCLCPP_INFO(this->get_logger(), "Target pose [x=%.2f, y=%.2f]",
                                        target_pose_.pose.position.x, target_pose_.pose.position.y);
    }
    
    void callbackTargetPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose_ = *msg;
    }

    // Callback to update the current pose and compute velocity
    void callbackCurrentTurtleSimPose(const turtlesim::msg::Pose::SharedPtr msg) {
        turtlesim_current_pose_ = *msg;

        // Convert Turtlesim Pose to PoseStamped
        current_pose_.header.stamp = this->get_clock()->now();
        current_pose_.header.frame_id = "map";

        // Populate position
        current_pose_.pose.position.x = turtlesim_current_pose_.x;
        current_pose_.pose.position.y = turtlesim_current_pose_.y;
        current_pose_.pose.position.z = 0.0;

        // Convert theta (yaw) to quaternion for orientation
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, turtlesim_current_pose_.theta);
        current_pose_.pose.orientation.x = quat.x();
        current_pose_.pose.orientation.y = quat.y();
        current_pose_.pose.orientation.z = quat.z();
        current_pose_.pose.orientation.w = quat.w();

        if (update_prior_current_pose_)
        {
            prior_current_pose_ = current_pose_;

            // Log for debugging
            RCLCPP_INFO(this->get_logger(), "Prior Current pose [x=%.2f, y=%.2f]",
                                            prior_current_pose_.pose.position.x, prior_current_pose_.pose.position.y);
            update_prior_current_pose_ = false;
        }
    }

    // Compute velocity commands and publish them
    void controlLoop() {
        
        // Compute distance and angle errors
        double dx_world = target_pose_.pose.position.x - current_pose_.pose.position.x;
        double dy_world = target_pose_.pose.position.y - current_pose_.pose.position.y;
        double distance = std::sqrt(dx_world * dx_world + dy_world * dy_world);

        auto cmd_msg = geometry_msgs::msg::Twist();
        if (distance > distance_threshold_ && updated_target_available_)
        {
            tf2::Quaternion quat(current_pose_.pose.orientation.x, current_pose_.pose.orientation.y,
                                 current_pose_.pose.orientation.z, current_pose_.pose.orientation.w);
            double roll, pitch, yaw_current;
            tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw_current);

            tf2::Quaternion q_target(target_pose_.pose.orientation.x, 
                                     target_pose_.pose.orientation.y,
                                     target_pose_.pose.orientation.z, 
                                     target_pose_.pose.orientation.w);
            double roll_target, pitch_target, yaw_target;
            tf2::Matrix3x3(q_target).getRPY(roll_target, pitch_target, yaw_target);

            // Compute theta error
            // double target_theta = std::atan2(dy_world, dx_world);
            double theta_error = yaw_target - yaw_current;
            while (theta_error > M_PI) { theta_error -= 2 * M_PI; }
            while (theta_error < -M_PI){ theta_error += 2 * M_PI; }

            if (std::abs(theta_error) > angle_threshold_) {
                cmd_msg.linear.x = 0.0;
                cmd_msg.angular.z = kp_angular_ * theta_error;
            } 
            else {
                cmd_msg.linear.x = kp_linear_ * distance;
                cmd_msg.angular.z = 0.0;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Target reached.\n");
            cmd_msg.linear.x = 0.0;
            cmd_msg.angular.z = 0.0;
            callGetTargetPose();
            prior_current_pose_ = current_pose_;
        }
        cmd_vel_pub_->publish(cmd_msg);
    }

    // Subscribers and Publishers
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr current_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    // Service Client
    rclcpp::Client<interfaces::srv::GetTargetPose>::SharedPtr get_target_pose_client_;

    // State Variables
    geometry_msgs::msg::PoseStamped target_pose_;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::PoseStamped prior_current_pose_;
    turtlesim::msg::Pose turtlesim_current_pose_;

    bool update_prior_current_pose_;
    bool updated_target_available_;

    // Control parameters
    double kp_linear_;
    double kp_angular_;
    double distance_threshold_;
    double angle_threshold_;
    double control_loop_timer_period_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
