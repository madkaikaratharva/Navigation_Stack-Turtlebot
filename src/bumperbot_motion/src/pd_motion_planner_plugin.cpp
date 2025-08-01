#include <chrono>
#include <math.h>
#include <algorithm>

#include "bumperbot_motion/pd_motion_planner_plugin.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Need to declare node parameters
#include "nav2_util/node_utils.hpp"

using std::placeholders::_1;

namespace bumperbot_motion
{
/*PDMotionPlanner::PDMotionPlanner()
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

}*/

void PDMotionPlanner::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                            std::shared_ptr<tf2_ros::Buffer>tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    // Initialize the parameters
    node_ = parent;
    auto node = node_.lock();
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;

    // Initialize the and clock
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Declare parameters for the server nodes
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kp", rclcpp::ParameterValue(2.0));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".kd", rclcpp::ParameterValue(0.1));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".step_size", rclcpp::ParameterValue(0.2));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));

    // Read run-time values of the declared parameters
    node->get_parameter(plugin_name_ + ".kp", kp_);
    node->get_parameter(plugin_name_ + ".kd", kd_);
    node->get_parameter(plugin_name_ + ".step_size", step_size_);
    node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
    node->get_parameter(plugin_name_ + ".max_linear_vel", max_angular_vel_);
    prev_linear_error_ = 0.0;
    prev_angular_error_  = 0.0;

    // Need this for debugging purpose
    next_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/pd/next_pose", 10);

}

void PDMotionPlanner::cleanup()
{
    RCLCPP_INFO(logger_, "Cleaning up plugin PDMotionPLanner");
    // Shutdown the publishers and subscribers if available
    next_pose_pub_.reset();
}

void PDMotionPlanner::activate()
{
    RCLCPP_INFO(logger_, "Activating the plugin PDMotionPlanner");
    last_cycle_time_ = clock_->now();
}

void PDMotionPlanner::deactivate()
{
    RCLCPP_INFO(logger_, "Deactivating the plugin PDMotionPlanner");
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
    transform = tf_->lookupTransform(frame, global_path_.header.frame_id, tf2::TimePointZero);
    }
    catch(tf2::TransformException& ex)
    {
        RCLCPP_WARN_STREAM(logger_, "The transform between " << frame << " and " << global_path_.header.frame_id << " does not exist!");
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

// Function to get the Global Path
void PDMotionPlanner::setPlan(const nav_msgs::msg::Path& path)
{
    global_path_ = path;
}

void PDMotionPlanner::setSpeedLimit(const double&, const bool&)
{

}

// Main Control Loop
geometry_msgs::msg::TwistStamped PDMotionPlanner::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                         const geometry_msgs::msg::Twist&,
                                                         nav2_core::GoalChecker*)

{

    // Create a new TwistStamped Message
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = robot_pose.header.frame_id;

    // If no path, return empty message
    if(global_path_.poses.empty())
    {
        RCLCPP_ERROR(logger_, "Empty Plan");
        return cmd_vel;
    }

    // Get the current position of the robot
    /*geometry_msgs::msg::TransformStamped robot_pose;
    try
    {
        robot_pose = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    }
    catch(tf2::TransformException& ex)
    {
        RCLCPP_WARN(get_logger(), "No Transform found between frame: odom and frame: base_footprint");
        RCLCPP_WARN(get_logger(), ex.what());
        return;
    }*/

    //RCLCPP_INFO_STREAM(get_logger(), "Robot Pose frame Id: " << robot_pose.header.frame_id.c_str());
    //RCLCPP_INFO_STREAM(get_logger(), "Global Path frame Id: " << global_path_.header.frame_id.c_str());


    // Transform Global Path to Odom Frame
    if(!transformPlan(robot_pose.header.frame_id))
    {
        RCLCPP_ERROR(logger_, "Unable to transform path in Robot's pose");
        return cmd_vel;
    }
    
    // Note: Robot pose recieved is already a PoseStamped Message
    // Calculate Next Pose on the Path to reach
    // Convert TransformedStamped Pose robot pose to PoseStamped message
    //geometry_msgs::msg::PoseStamped robot_pose_stamped;
    //robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
    //robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
    //robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
    //robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;

    //geometry_msgs::msg::PoseStamped next_pose = getNextPose(robot_pose);

    // Note: Goal Checker will be carried out by the Controller Server
    // Check if the robot has reached the point
    //double dx = next_pose.pose.position.x - robot_pose_stamped.pose.position.x;
    //double dy = next_pose.pose.position.y - robot_pose_stamped.pose.position.y;
    //double distance = std::sqrt(dx * dx + dy * dy);

    //if(distance <= 0.06)
    //{
    //    RCLCPP_INFO(get_logger(), "Goal Reached");
    //    global_path_.poses.clear();
    //    return;
    //}

    // Publish the next pose
    geometry_msgs::msg::PoseStamped next_pose = getNextPose(robot_pose);
    next_pose_pub_->publish(next_pose);

    tf2::Transform robot_pose_tf, next_pose_tf;
    tf2::fromMsg(robot_pose.pose, robot_pose_tf); 
    tf2::fromMsg(next_pose.pose, next_pose_tf);
    
    // Now we have (Todom,robot_pose) and (Todom,next_pose)
    tf2::Transform next_pose_robot_tf = robot_pose_tf.inverse() * next_pose_tf;

    // Now calculate the dx, dy, dtheta
    double linear_error = next_pose_robot_tf.getOrigin().getX();
    double angular_error = next_pose_robot_tf.getOrigin().getY();

    // Derivate error for Kd gain
    double dt = (clock_->now() - last_cycle_time_).seconds();
    double linear_err_derivative = (linear_error - prev_linear_error_) / dt;
    double angular_err_derivative = (angular_error - prev_angular_error_) / dt;

    // Create a new velocity message
    //geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.twist.linear.x = std::clamp(kp_ * linear_error + kd_ * linear_err_derivative, -max_linear_vel_, max_linear_vel_);
    cmd_vel.twist.angular.z = std::clamp(kp_ * angular_error + kd_ * angular_err_derivative, -max_angular_vel_, max_angular_vel_);
    //cmd_pub_->publish(cmd_vel);

    //
    prev_linear_error_ = linear_error;
    prev_angular_error_ = angular_error;
    last_cycle_time_ = clock_->now();

    return cmd_vel;
}
}

// Note: No need of Main function as this is a plugin and not an executable
/*int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PDMotionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}*/

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bumperbot_motion::PDMotionPlanner, nav2_core::Controller)