#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace bumperbot_motion
{
class PurePursuit : public rclcpp::Node
{
private:
    // subscription to receive Path from the planner
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    // publisher to publish velocity commands
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // publisher for visualization to publish next pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pose_pub_;

    // Create Instances of Buffer and Transform Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS2 timer for executing the controller loop
    rclcpp::TimerBase::SharedPtr control_loop_;

    // Parameters for Pure Pursuit controller
    double look_ahead_distance_;

    // Parameters for Motion Planner
    double max_linear_vel_;
    double max_angular_vel_;

    // Path Message. Gobal Path
    nav_msgs::msg::Path global_path_;

public:
    PurePursuit();

private:

    // Control Loop function
    void controlLoop();

    // Path Callback
    void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

    // Transform the global path to the given frame
    bool transformPlan(const std::string& frame);

    // Returns the carrot point / pose on the path
    geometry_msgs::msg::PoseStamped getCarrotPose(const geometry_msgs::msg::PoseStamped& robot_pose);

    // Determine the Curvatue
    double getCurvature(const geometry_msgs::msg::Pose& carrot_pose);

};
}