#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Include the controller template class from Nav2
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace bumperbot_motion
{
class PDMotionPlanner : public nav2_core::Controller
{
private:
    // subscription to receive Path from the planner
    //rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    // publisher to publish velocity commands
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // publisher for visualization to publish next pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_pose_pub_;

    // Create Instances of Buffer and Transform Listener
    //std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    //std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS2 timer for executing the controller loop
    //rclcpp::TimerBase::SharedPtr control_loop_;

    // Parameters for PD controller
    double kp_;
    double kd_;
    double step_size_;

    // Parameters for Motion Planner
    double max_linear_vel_;
    double max_angular_vel_;

    // Path Message. Gobal Path
    nav_msgs::msg::Path global_path_;

    // Error
    double prev_linear_error_;
    double prev_angular_error_;
    rclcpp::Time last_cycle_time_;

    // New variables
    std::string plugin_name_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    rclcpp::Logger logger_{rclcpp::get_logger("PDMotionplanner")};
    rclcpp::Clock::SharedPtr clock_;
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;


public:
    PDMotionPlanner() = default;
    ~PDMotionPlanner() = default;

    // Override the function behavior in the child class
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                    std::shared_ptr<tf2_ros::Buffer>tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
    void cleanup() override;
    void activate() override;
    void deactivate() override;

    // Control Loop 
    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                                const geometry_msgs::msg::Twist& velocity,
                                                                nav2_core::GoalChecker* goal_checker) override;
    

    void setPlan(const nav_msgs::msg::Path& path) override;
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

private:    

    // Control Loop function
    //void controlLoop();

    // Path Callback
    //void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

    // Transform the global path to the given frame
    bool transformPlan(const std::string& frame);

    // returns next pose in the path that robot will try to reach
    geometry_msgs::msg::PoseStamped getNextPose(const geometry_msgs::msg::PoseStamped& robot_pose);

};
}