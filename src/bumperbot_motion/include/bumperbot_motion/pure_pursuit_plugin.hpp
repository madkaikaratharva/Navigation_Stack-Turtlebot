#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Include Declaration of the Base Class
#include "nav2_core/controller.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace bumperbot_motion
{
class PurePursuit : public nav2_core::Controller
{
private:
    // subscription to receive Path from the planner
    //rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

    // publisher to publish velocity commands
    //rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    // Create Instances of Buffer and Transform Listener
    //std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    //std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // ROS2 timer for executing the controller loop
    //rclcpp::TimerBase::SharedPtr control_loop_;

    // publisher for visualization to publish next pose
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr carrot_pose_pub_;

    // Parameters for Pure Pursuit controller
    double look_ahead_distance_;

    // Parameters for Motion Planner
    double max_linear_vel_;
    double max_angular_vel_;

    // Path Message. Gobal Path
    nav_msgs::msg::Path global_path_;

    // Variables for the Base Class //
    // Plugin Name
    std::string plugin_name_;

    // Stores 2D Costmap. Local Costmap
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    // Logger variable from the base class
    rclcpp::Logger logger_{rclcpp::get_logger("PurePursuit")};

    // Ros2 clock from the based class
    rclcpp::Clock::SharedPtr clock_;

    // Pointer to te node
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    // TF Buffer provided by the base class
    std::shared_ptr<tf2_ros::Buffer> tf_;

public:
    PurePursuit() = default;
    ~PurePursuit() = default;

    // Override the behavior of base class functions //
    // Configure function for Lifecycle Node
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                    std::shared_ptr<tf2_ros::Buffer>tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    
    // Functions of Lifecycle Node
    void cleanup() override;
    void activate() override;
    void deactivate() override;

    // Control Loop 
    geometry_msgs::msg::TwistStamped computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                                const geometry_msgs::msg::Twist& velocity,
                                                                nav2_core::GoalChecker* goal_checker) override;
    
    // Function to receive the Global Plan/Path
    void setPlan(const nav_msgs::msg::Path& path) override;

    // Sets Speed limits
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override;

private:

    // Control Loop function
    //void controlLoop();

    // Path Callback
    //void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

    // Transform the global path to the given frame
    bool transformPlan(const std::string& frame);

    // Returns the carrot point / pose on the path
    geometry_msgs::msg::PoseStamped getCarrotPose(const geometry_msgs::msg::PoseStamped& robot_pose);

    // Determine the Curvatue
    double getCurvature(const geometry_msgs::msg::Pose& carrot_pose);

};
}