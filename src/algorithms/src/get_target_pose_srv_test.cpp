#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/get_target_pose.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("get_target_pose_srv_test");
    
    auto client = node->create_client<interfaces::srv::GetTargetPose>("get_target_pose");
    while (!client->wait_for_service(1s))
    {
        RCLCPP_WARN(node->get_logger(), "Waiting for the server...");
    }

    auto request = std::make_shared<interfaces::srv::GetTargetPose::Request>();
    request->need_new_target = true;

    auto future = client->async_send_request(request);
    rclcpp::spin_until_future_complete(node, future);

    auto response = future.get();
    if (response->updated_target)
    {
        RCLCPP_INFO(node->get_logger(), "Updated target pose");
    }
    else
    {
        RCLCPP_INFO(node->get_logger(), "No updated target pose");
    }

    rclcpp::shutdown();
    return 0;
}