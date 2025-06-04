#pragma once

#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

namespace bumperbot_mapping{

inline const double PRIOR_PROB = 0.5;   // Prior Probability of each cell
inline const double OCC_PROB = 0.9;     // Use this value to mark the probability of occupied cell
inline const double FREE_PROB = 0.35;

struct Pose
{
    int x, y;
    Pose() = default;
    Pose(const int px, const int py) : x(px), y(py) {};
};

// Maps Pose ----> Map Grid Cell
unsigned int poseToCell(const Pose& pose, const nav_msgs::msg::MapMetaData& map_info);

// TF2 transform ----> Pose
Pose coordinatestoPose(const double px, const double py, const nav_msgs::msg::MapMetaData& map_info);

// Verify is Pose is in the map bounds
bool poseOnMap(const Pose& pose, const nav_msgs::msg::MapMetaData& map_info);

// Bresenham's Algorithm
std::vector<Pose> bresenham(const Pose& start, const Pose& end);

// Function to Map free and occupied cells on the map
std::vector<std::pair<Pose, double>> inverseSensorModel(const Pose& robot_pose, const Pose& beam_pose);

// Probability to LogOdds Function
double prob2logodds(double prob);

// LogOdds to Probability
double logodds2prob(double log);

class MappingWithKnownPoses : public rclcpp::Node
{
private:
    // Create an instance of occupancy grid message
    nav_msgs::msg::OccupancyGrid map_;

    // New Variable to generate a Probability Map. Each cell contains occupancy probability of each cell
    std::vector<double> probability_map_;

    // Recieve messages from Lidar Sensor
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    // Publisher to publish Map
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // Timer Object
    rclcpp::TimerBase::SharedPtr timer_;

    // Get Pose of the robot using the TF2 library
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = {nullptr};

public:
    MappingWithKnownPoses(const char* name);

private:
    // Takes Laser callback message
    void scanCallback(const sensor_msgs::msg::LaserScan& scan);

    void timerCallback();
};
}