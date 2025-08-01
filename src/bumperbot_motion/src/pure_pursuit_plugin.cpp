#include <chrono>
#include <math.h>
#include <algorithm>

#include "bumperbot_motion/pure_pursuit_plugin.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Need to declare node parameters
#include "nav2_util/node_utils.hpp"

using std::placeholders::_1;

namespace bumperbot_motion
{
/*PurePursuit::PurePursuit()
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
}*/

void PurePursuit::configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
                std::shared_ptr<tf2_ros::Buffer>tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    // Initialize the Base class params
    node_ = parent.lock();
    auto node = node_.lock();
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;

    // Initialize the and clock
    logger_ = node->get_logger();
    clock_ = node->get_clock();

    // Declare parameters for the server nodes
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".look_ahead_distance", rclcpp::ParameterValue(0.5));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_linear_vel", rclcpp::ParameterValue(0.3));
    nav2_util::declare_parameter_if_not_declared(node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(1.0));

    // Read run-time values of the declared parameters
    node->get_parameter(plugin_name_ + ".look_ahead_distance", look_ahead_distance_);
    node->get_parameter(plugin_name_ + ".max_linear_vel", max_linear_vel_);
    node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);

    // Publisher to publish te next planned pose
    carrot_pose_pub_ = node->create_publisher<geometry_msgs::msg::PoseStamped>("/pure_pursuit/carrot_pose", 10);

}

void PurePursuit::cleanup()
{
    RCLCPP_INFO(logger_, "Cleaning up plugin: PurePursuit");
    // Shutdown the publishers and subscribers if available
    carrot_pose_pub_.reset();
}

void PurePursuit::activate() 
{
    RCLCPP_INFO(logger_, "Activating the plugin: PurePursuit");
}

void PurePursuit::deactivate()
{
    RCLCPP_INFO(logger_, "Deactivating the plugin: PurePursuit");
}

// Gets the Global Path
void PurePursuit::setPlan(const nav_msgs::msg::Path& path)
{
    global_path_ = path;
}

void PurePursuit::setSpeedLimit(const double&, const bool&)
{

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
geometry_msgs::msg::TwistStamped PurePursuit::computeVelocityCommands(const geometry_msgs::msg::PoseStamped& robot_pose,
                                                                      const geometry_msgs::msg::Twist&,
                                                                      nav2_core::GoalChecker*)
{

    // Create a new TwistStamped Message to publish velocity commands
    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = robot_pose.header.frame_id;

    // Return empty cmd_vel if no path available
    if(global_path_.poses.empty())
    {
        return cmd_vel;
    }

    // Note: Controller Server provides robot pose w.r.t odom frame
    // Get the current position of the robot
    //geometry_msgs::msg::TransformStamped robot_pose;
    //try
    //{
    //    robot_pose = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
    //}
    //catch(tf2::TransformException& ex)
    //{
    //    RCLCPP_WARN(get_logger(), "No Transform found between frame: odom and frame: base_footprint");
    //    RCLCPP_WARN(get_logger(), ex.what());
    //    return;
    //}
    //RCLCPP_INFO_STREAM(get_logger(), "Robot Pose frame Id: " << robot_pose.header.frame_id.c_str());
    //RCLCPP_INFO_STREAM(get_logger(), "Global Path frame Id: " << global_path_.header.frame_id.c_str());


    // Transform Global Path to Odom Frame
    if(!transformPlan(robot_pose.header.frame_id))
    {
        RCLCPP_ERROR(logger_, "Unable to transform path in Robot's pose");
        return cmd_vel;
    }

    // Calculate Next Pose on the Path to reach //

    // Convert TransformedStamped Pose robot pose to PoseStamped message
    //geometry_msgs::msg::PoseStamped robot_pose_stamped;
    //robot_pose_stamped.header.frame_id = robot_pose.header.frame_id;
    //robot_pose_stamped.pose.position.x = robot_pose.transform.translation.x;
    //robot_pose_stamped.pose.position.y = robot_pose.transform.translation.y;
    //robot_pose_stamped.pose.orientation = robot_pose.transform.rotation;


    geometry_msgs::msg::PoseStamped carrot_pose = getCarrotPose(robot_pose);

    // Note: Controller Server carries out the check //
    // Check if the robot has reached the point
    //double dx = carrot_pose.pose.position.x - robot_pose.pose.position.x;
    //double dy = carrot_pose.pose.position.y - robot_pose.pose.position.y;
    //double distance = std::sqrt(dx * dx + dy * dy);
    //if(distance <= 0.1)
    //{
    //    RCLCPP_INFO(get_logger(), "Goal Reached");
    //    global_path_.poses.clear();
    //    return;
    //}


    // Publish the next pose
    carrot_pose_pub_->publish(carrot_pose);

    tf2::Transform robot_pose_tf, carrot_pose_tf;
    tf2::fromMsg(robot_pose.pose, robot_pose_tf); 
    tf2::fromMsg(carrot_pose.pose, carrot_pose_tf);
    
    // Now we have (Todom,robot_pose) and (Todom,next_pose)
    // This transform provides Pose of the the Waypoint in Robot's frame
    tf2::Transform carrot_pose_robot_tf = robot_pose_tf.inverse() * carrot_pose_tf;

    // Covert the Carrot Pose Transform object to Pose Message
    tf2::toMsg(carrot_pose_robot_tf, carrot_pose.pose);

    // Calculate the Curvature
    double curvature = getCurvature(carrot_pose.pose);

    // Now generate velocity commands based on the curvature
    //geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.twist.linear.x = max_linear_vel_;
    cmd_vel.twist.angular.z = std::clamp(curvature * max_angular_vel_, -max_angular_vel_, max_angular_vel_);
    //cmd_pub_->publish(cmd_vel);

    return cmd_vel;

}
}

/*int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_motion::PurePursuit>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}*/

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bumperbot_motion::PurePursuit, nav2_core::Controller)