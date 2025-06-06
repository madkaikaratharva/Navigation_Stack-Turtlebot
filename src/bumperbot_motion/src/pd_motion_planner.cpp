#include <chrono>
#include <math.h>
#include <algorithm>

#include "bumperbot_motion/pd_motion_planner.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

namespace bumperbot_motion
{
PDMotionPlanner::PDMotionPlanner()
    : Node("pd_motion_planner_node")
    , kp_(2.0)
    , kd_(0.1)
    , step_size_(0.2)
    , max_linear_vel_(0.3)
    , max_angular_vel_(1.0)
    , prev_linear_error_(0.0)
    , prev_angular_error_(0.0)
{
    // Initialize parameters
    declare_parameter<double>("kp", kp_);
    declare_parameter<double>("kd", kd_);
    declare_parameter<double>("step_size", step_size_);

    declare_parameter<double>("max_linear_vel", max_linear_vel_);
    declare_parameter<double>("max_angular_vel", max_angular_vel_);

    // Read run-time value of the node parameters
    kp_ = get_parameter("kp").as_double();
    kd_ = get_parameter("kd").as_double();
    step_size_ = get_parameter("step_size").as_double();

    max_linear_vel_ = get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = get_parameter("max_angular_vel").as_double();

    // Initialize the subsribers and publishers
    path_sub_ = create_subscription<nav_msgs::msg::Path>("/astar/path", 10,
                                                            std::bind(&PDMotionPlanner::pathCallback, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    next_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);

    // Initialize Buffer and Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    control_loop_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PDMotionPlanner::controlLoop, this));
    last_cycle_time = get_clock()->now();

}

// Path Callback function
void PDMotionPlanner::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    // Get the path
    global_path_ = *path;
}

// Transform Path to provided frame ID
bool PDMotionPlanner::transformPlan(const std::string& frame)
{
    if(global_path_.header.frame_id == frame){
        return true;
    }

    // Retreive the relative transformation
    geometry_msgs::msg::TransformStamped transform; 
    try
    { 
    transform = tf_buffer_->lookupTransform(frame, global_path_.header.frame_id, tf2::TimePointZero);
    }
    catch(tf2::TransformException& ex)
    {
        RCLCPP_WARN_STREAM(get_logger(), "The transform between " << frame << " and " << global_path_.header.frame_id << " does not exist!");
        return false;
    }

    for(auto& pose : global_path_.poses)
    {
        // Convert Geometry Messages (Pose) to TF2 objects
        tf2::doTransform(pose, pose, transform);

    }

    global_path_.header.frame_id = frame;
    return true;
    
}

geometry_msgs::msg::PoseStamped PDMotionPlanner::getNextPose(const geometry_msgs::msg::PoseStamped& robot_pose)
{
    geometry_msgs::msg::PoseStamped next_pose = global_path_.poses.back();

    for(auto pose_it = global_path_.poses.rbegin(); pose_it != global_path_.poses.rend(); ++pose_it)
    {
        // Calculate distance between path poses and robot pose
        double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
        double dy = pose_it->pose.position.y - robot_pose.pose.position.y;

        double distance = std::sqrt(dx * dx + dy * dy);
        if(distance > step_size_)
        {
            next_pose = *pose_it;
            continue;
        }
        
        next_pose = *pose_it;
        break;
    }

    return next_pose;
}

// Main Control Loop
void PDMotionPlanner::controlLoop()
{
    if(global_path_.poses.empty())
    {
        return;
    }

    // Get the current position of the robot
    geometry_msgs::msg::TransformStamped robot_pose;
    try
    {
        robot_pose = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    }
    catch(tf2::TransformException& ex)
    {
        RCLCPP_WARN(get_logger(), "No Transform found between frame: odom and frame: base_footprint");
        RCLCPP_WARN(get_logger(), ex.what());
        return;
    }

    //RCLCPP_INFO_STREAM(get_logger(), "Robot Pose frame Id: " << robot_pose.header.frame_id.c_str());
    //RCLCPP_INFO_STREAM(get_logger(), "Global Path frame Id: " << global_path_.header.frame_id.c_str());

    // Transform Global Path to Odom Frame
    if(!transformPlan(robot_pose.header.frame_id))
    {
        RCLCPP_ERROR(get_logger(), "Unable to transform path in Robot's pose");
        return;
    }

    // Calculate Next Pose on the Path to reach
    // Convert TransformedStamped Pose robot pose to PoseStamped message
    geometry_msgs::msg::PoseStamped robot_pose_stamped;
    robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
    robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
    robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
    robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;

    geometry_msgs::msg::PoseStamped next_pose = getNextPose(robot_pose_stamped);

    // Check if the robot has reached the point
    double dx = next_pose.pose.position.x - robot_pose_stamped.pose.position.x;
    double dy = next_pose.pose.position.y - robot_pose_stamped.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if(distance <= 0.06)
    {
        RCLCPP_INFO(get_logger(), "Goal Reached");
        global_path_.poses.clear();
        return;
    }

    // Publish the next pose
    next_pose_pub_->publish(next_pose);

    tf2::Transform robot_pose_tf, next_pose_tf;
    tf2::fromMsg(robot_pose_stamped.pose, robot_pose_tf); 
    tf2::fromMsg(next_pose.pose, next_pose_tf);
    
    // Now we have (Todom,robot_pose) and (Todom,next_pose)
    tf2::Transform next_pose_robot_tf = robot_pose_tf.inverse() * next_pose_tf;

    // Now calculate the dx, dy, dtheta
    double linear_error = next_pose_robot_tf.getOrigin().getX();
    double angular_error = next_pose_robot_tf.getOrigin().getY();

    // Derivate error for Kd gain
    double dt = (get_clock()->now() - last_cycle_time).seconds();
    double linear_err_derivative = (linear_error - prev_linear_error_) / dt;
    double angular_err_derivative = (angular_error - prev_angular_error_) / dt;

    // Create a new velocity message
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_err_derivative, -max_linear_vel_, max_linear_vel_);
    cmd_vel.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_err_derivative, -max_angular_vel_, max_angular_vel_);
    cmd_pub_->publish(cmd_vel);

    //
    prev_linear_error_ = linear_error;
    prev_angular_error_ = angular_error;
    last_cycle_time = get_clock()->now();

}
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PDMotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}