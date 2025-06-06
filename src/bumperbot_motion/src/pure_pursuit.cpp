#include <chrono>
#include <math.h>
#include <algorithm>

#include "bumperbot_motion/pure_pursuit.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

namespace bumperbot_motion
{
PurePursuit::PurePursuit()
    : Node("pd_motion_planner_node")
    , look_ahead_distance_(0.5)
    , max_linear_vel_(0.3)
    , max_angular_vel_(1.0)
{
    // Initialize parameters
    declare_parameter<double>("look_ahead_distance", look_ahead_distance_);
    declare_parameter<double>("max_linear_vel", max_linear_vel_);
    declare_parameter<double>("max_angular_vel", max_angular_vel_);

    // Read run-time value of the node parameters
    look_ahead_distance_ = get_parameter("look_ahead_distance").as_double();
    max_linear_vel_ = get_parameter("max_linear_vel").as_double();
    max_angular_vel_ = get_parameter("max_angular_vel").as_double();

    // Initialize the subsribers and publishers
    path_sub_ = create_subscription<nav_msgs::msg::Path>("/astar/path", 10,
                                                            std::bind(&PurePursuit::pathCallback, this, _1));

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    carrot_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/carrot_pose", 10);

    // Initialize Buffer and Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    control_loop_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&PurePursuit::controlLoop, this));

}

// Path Callback function
void PurePursuit::pathCallback(const nav_msgs::msg::Path::SharedPtr path)
{
    // Get the path
    global_path_ = *path;
}

// Transform Path to provided frame ID
bool PurePursuit::transformPlan(const std::string& frame)
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

geometry_msgs::msg::PoseStamped PurePursuit::getCarrotPose(const geometry_msgs::msg::PoseStamped& robot_pose)
{
    geometry_msgs::msg::PoseStamped carrot_pose = global_path_.poses.back();

    for(auto pose_it = global_path_.poses.rbegin(); pose_it != global_path_.poses.rend(); ++pose_it)
    {
        // Calculate distance between path poses and robot pose
        double dx = pose_it->pose.position.x - robot_pose.pose.position.x;
        double dy = pose_it->pose.position.y - robot_pose.pose.position.y;

        double distance = std::sqrt(dx * dx + dy * dy);
        if(distance > look_ahead_distance_)
        {
            carrot_pose = *pose_it;
            continue;
        }
        
        carrot_pose = *pose_it;
        break;
    }

    return carrot_pose;
}

double PurePursuit::getCurvature(const geometry_msgs::msg::Pose& carrot_pose)
{
    // Calcluate the Euclidean distance between Carrot and the Robot
    const double L = (carrot_pose.position.x * carrot_pose.position.x) + (carrot_pose.position.y * carrot_pose.position.y);

    if(L > 0.001)
    {
        return (2.0 * carrot_pose.position.y) / L;
    }
    else
    {
        return 0.0;
    }
}

// Main Control Loop
void PurePursuit::controlLoop()
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

    geometry_msgs::msg::PoseStamped carrot_pose = getCarrotPose(robot_pose_stamped);

    // Check if the robot has reached the point
    double dx = carrot_pose.pose.position.x - robot_pose_stamped.pose.position.x;
    double dy = carrot_pose.pose.position.y - robot_pose_stamped.pose.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    if(distance <= 0.1)
    {
        RCLCPP_INFO(get_logger(), "Goal Reached");
        global_path_.poses.clear();
        return;
    }

    // Publish the next pose
    carrot_pose_pub_->publish(carrot_pose);

    tf2::Transform robot_pose_tf, carrot_pose_tf;
    tf2::fromMsg(robot_pose_stamped.pose, robot_pose_tf); 
    tf2::fromMsg(carrot_pose.pose, carrot_pose_tf);
    
    // Now we have (Todom,robot_pose) and (Todom,next_pose)
    // This transform provides Pose of the the Waypoint in Robot's frame
    tf2::Transform carrot_pose_robot_tf = robot_pose_tf.inverse() * carrot_pose_tf;

    // Covert the Carrot Pose Transform object to Pose Message
    tf2::toMsg(carrot_pose_robot_tf, carrot_pose.pose);

    // Calculate the Curvature
    double curvature = getCurvature(carrot_pose.pose);

    // Now generate velocity commands based on the curvature
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = max_linear_vel_;
    cmd_vel.angular.z = std::clamp(curvature * max_angular_vel_, -max_angular_vel_, max_angular_vel_);
    cmd_pub_->publish(cmd_vel);

}
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}