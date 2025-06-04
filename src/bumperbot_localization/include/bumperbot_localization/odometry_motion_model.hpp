#ifndef ODOMETRY_MOTION_MODEL_HPP
#define ODOMETRY_MOTION_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>

class OdometryMotionModel : public rclcpp::Node
{
private:

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;

    // Contains samples of all the possible positions of the robot
    geometry_msgs::msg::PoseArray samples_;

    // Constants for Motion model
    double alpha1_;
    double alpha2_;
    double alpha3_;
    double alpha4_;

    // Desired no. of particles to represent all possible position of robot
    int nr_samples;

    double last_odom_x_;
    double last_odom_y_;
    double last_odom_theta_;

    bool is_first_odom_;


    void odomCallback(const nav_msgs::msg::Odometry& odom);

public:

    OdometryMotionModel(const std::string& name);

};


#endif // ODOMETRY_MOTION_MODEL_HPP