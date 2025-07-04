#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/srv/get_target_pose.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class PoseManagerNode : public rclcpp::Node
{
public:
    PoseManagerNode() : Node("pose_manager")
                    , last_gui_pose_time_(this->get_clock()->now())
                    , use_gui_pose_(false)
                    , updated_active_pose_(false)
    {
        // Parameters
        this->declare_parameter("gui_timeout_seconds", 30.0);
        gui_timeout_seconds_ = this->get_parameter("gui_timeout_seconds").as_double();

        // Subscribers of clock pose
        gui_cli_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/gui_cli_pose", 10, std::bind(&PoseManagerNode::callbackGuiCliPose, this, _1));
        clock_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/clock_pose", 10, std::bind(&PoseManagerNode::callbackClockPose, this, _1));

        // Publisher for target pose
        target_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose", 10);

        // Service
        get_target_pose_service_ = this->create_service<interfaces::srv::GetTargetPose>(
            "get_target_pose",
            std::bind(&PoseManagerNode::callbackGetTargetPose, this, _1, _2));
        
        RCLCPP_INFO(this->get_logger(), "Pose Manager Node has started.");
    }
 
private:
    void callbackGetTargetPose(const interfaces::srv::GetTargetPose::Request::SharedPtr request,
                               const interfaces::srv::GetTargetPose::Response::SharedPtr response)
    {
        if (request->need_new_target)
        {
            RCLCPP_INFO(this->get_logger(), "Received new target pose request.");
            auto now = this->get_clock()->now();
            RCLCPP_INFO(this->get_logger(), "Time difference since last GUI update: %.2f", (now - last_gui_pose_time_).seconds());

            if ((now - last_gui_pose_time_).seconds() > gui_timeout_seconds_) 
            {
                RCLCPP_WARN(this->get_logger(), "Switching to clock pose due to GUI timeout");
                use_gui_pose_ = false;      // Switch to clock pose
                active_pose_ = clock_pose_; // Update the active pose to clock pose
                updated_active_pose_ = true;
            }

            if (!updated_active_pose_)
            {
                response->updated_target = false;
                RCLCPP_INFO(this->get_logger(), "Sorry! No new target pose yet.");
                return;
            }

            
            response->updated_target = true;
            publishActivePose();
            RCLCPP_INFO(this->get_logger(), "Published active target pose via service.");
        }
    }
    void callbackGuiCliPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        last_gui_pose_time_ = this->get_clock()->now();

        active_pose_ = *msg;
        updated_active_pose_ = true;
        use_gui_pose_ = true;   // gui pose is active pose
    }
    
    void callbackClockPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        clock_pose_ = *msg;
        if (!use_gui_pose_) 
        {
            active_pose_ = *msg;    // clock pose is active pose if gui pose isn't used
            updated_active_pose_ = true;
        }
    }

    void publishActivePose() 
    {
        // Publish the active pose
        auto msg = geometry_msgs::msg::PoseStamped();
        msg = active_pose_;
        // Log for debugging
        RCLCPP_INFO(this->get_logger(), "Active pose [x=%.2f, y=%.2f]",
                                        active_pose_.pose.position.x, active_pose_.pose.position.y);
        target_pose_pub_->publish(msg);
        updated_active_pose_ = false;
    }
    
    // Subscriber & Publisher
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr clock_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr gui_cli_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;

    // Service
    rclcpp::Service<interfaces::srv::GetTargetPose>::SharedPtr get_target_pose_service_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // State variables
    geometry_msgs::msg::PoseStamped clock_pose_;
    geometry_msgs::msg::PoseStamped active_pose_;
    rclcpp::Time last_gui_pose_time_;
    bool use_gui_pose_;
    bool updated_active_pose_;
    double gui_timeout_seconds_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PoseManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}