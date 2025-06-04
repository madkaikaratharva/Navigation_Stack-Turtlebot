#include <thread>
#include <memory>
#include <chrono>
#include <functional>

#include "bumperbot_mapping/mapping_with_known_poses.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/utils.h"

namespace bumperbot_mapping
{

using namespace std::placeholders;
using namespace std::chrono_literals;


Pose coordinatestoPose(const double px, const double py, const nav_msgs::msg::MapMetaData& map_info)
{
    Pose pose;
    pose.x = std::round((px - map_info.origin.position.x) / map_info.resolution); 
    pose.y = std::round((py - map_info.origin.position.y) / map_info.resolution); 
    return pose;
}

bool poseOnMap(const Pose& pose, const nav_msgs::msg::MapMetaData& map_info)
{
    return pose.x < static_cast<int>(map_info.width) && pose.x >=0 && pose.y < static_cast<int>(map_info.height) && pose.y >=0;
}

unsigned int poseToCell(const Pose& pose, const nav_msgs::msg::MapMetaData& map_info)
{
    return map_info.width * pose.y + pose.x;
}


std::vector<Pose> bresenham(const Pose& start, const Pose& end)
{
    std::vector<Pose> line;
    int dx = end.x - start.x;
    int dy = end.y - start.y;
    int xsign = dx > 0 ? 1 : -1;
    int ysign = dy > 0 ? 1 : -1;
    dx = std::abs(dx);
    dy = std::abs(dy);
    int xx, xy, yx, yy;

    if(dx > dy){
        xx = xsign;
        xy = 0;
        yx = 0;
        yy = ysign;
    }
    else{
        int tmp = dx;
        dx = dy;
        dy = tmp;
        xx = 0;
        xy = ysign;
        yx = xsign;
        yy = 0;
    }

    int D = 2 * dy - dx;
    int y = 0;
    line.reserve(dx + 1);

    for (int i = 0; i < dx + 1; i++)
    {
        line.emplace_back(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy));
        if(D >= 0)
        {
            y++;
            D -= 2 * dx;
        }
        D += 2 * dy;
    }

    return line;
}

std::vector<std::pair<Pose, double>> inverseSensorModel(const Pose& robot_pose, const Pose& beam_pose)
{
    std::vector<std::pair<Pose, double>> occupancy_values;
    std::vector<Pose> line = bresenham(robot_pose, beam_pose);
    occupancy_values.reserve(line.size());

    // Mark Intermediate cells as free on the map
    for(long unsigned int i=0; i < line.size()-1u; i++)
    {
        occupancy_values.emplace_back(std::pair<Pose, double>(line.at(i), FREE_PROB));
    }

    // Last Cell will be occupied.
    occupancy_values.emplace_back(std::pair<Pose, double>(line.back(), OCC_PROB));

    return occupancy_values;
}

double prob2logodds(double prob)
{
    return std::log(prob / (1 - prob));
}

double logodds2prob(double log)
{
    return (1 - (1 / (1 + std::exp(log))));
}



MappingWithKnownPoses::MappingWithKnownPoses(const char* name)
    : Node(name)
{
    // Declare Parameters of this Node
    declare_parameter<double>("width", 50.0);
    declare_parameter<double>("height", 50.0);
    declare_parameter<double>("resolution", 0.1);

    double width = get_parameter("width").as_double();
    double height = get_parameter("height").as_double();
    double resolution = get_parameter("resolution").as_double();
    
    // Initialize the Map Message
    map_.info.resolution = resolution;
    map_.info.width = std::round(width / resolution);   // Takes the number of cells
    map_.info.height = std::round(height / resolution); // Takes the number of cells
    map_.info.origin.position.x = - std::round(width / 2.0);
    map_.info.origin.position.y = - std::round(height / 2.0);
    map_.header.frame_id = "odom";  // Indicates that we will use only odometry to create the map.
    map_.data = std::vector<int8_t>(map_.info.width * map_.info.height, -1);    // '-1' indicates unknown value


    // Initialize the Probability Distribution Map
    probability_map_ = std::vector<double>(map_.info.height * map_.info.width, prob2logodds(PRIOR_PROB));

    // Initialize the Publishers, Subscribers, Timers
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, 
                                                                    std::bind(&MappingWithKnownPoses::scanCallback, this, _1));
    timer_ = create_wall_timer(1s, std::bind(&MappingWithKnownPoses::timerCallback, this));

    // TF2 Buffer Object between frame odom (reference frame) and laser frame
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void MappingWithKnownPoses::scanCallback(const sensor_msgs::msg::LaserScan& scan)
{
    // Read the current position of the robot. tf2::TimePointZero provides the latest available transform
    geometry_msgs::msg::TransformStamped t;
    try{
        t = tf_buffer_->lookupTransform(map_.header.frame_id, scan.header.frame_id, tf2::TimePointZero);
    } 
    catch(const tf2::TransformException& execption){
        RCLCPP_ERROR(get_logger(), "Unable to get transform between /odom and /base_footprint");
        return;
    }

    // Convert TransformStamped message to Pose message
    Pose robot_pose = coordinatestoPose(t.transform.translation.x, t.transform.translation.y, map_.info);
    
    if(!poseOnMap(robot_pose, map_.info))
    {
        RCLCPP_ERROR(get_logger(), "Robot is out of map bounds");
        return;
    }

    tf2::Quaternion q(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Iterate through the scan messages
    for(long unsigned int i = 0; i < scan.ranges.size(); i++)
    {
        // Polar to Cartesian. px, py, theta are expressed in 'odom' fixed frame
        double angle = scan.angle_min + (i * scan.angle_increment) + yaw;
        double px = (scan.ranges.at(i) * std::cos(angle)) + t.transform.translation.x;
        double py = (scan.ranges.at(i) * std::sin(angle)) + t.transform.translation.y;

        // Obstacle Pose in Map
        Pose beam_pose = coordinatestoPose(px, py, map_.info);
        if(!poseOnMap(beam_pose, map_.info)){
            continue;
        }
        else{
            std::vector<std::pair<Pose, double>> poses = inverseSensorModel(robot_pose, beam_pose);
            for(const auto& pose : poses)
            {
                // Convert the Pose to Grid Cell
                unsigned int cell = poseToCell(pose.first, map_.info);
                //map_.data.at(cell) = pose.second;

                // Update the Probability Map
                probability_map_.at(cell) += prob2logodds(pose.second) - prob2logodds(PRIOR_PROB);
            }
            //unsigned beam_cell = poseToCell(beam_pose, map_.info);
            //map_.data.at(beam_cell) = 100; 
        }
    }

    //unsigned int robot_cell = poseToCell(robot_pose, map_.info);
    //map_.data.at(robot_cell) = 100; // Marking the occupied cell as black
}

void MappingWithKnownPoses::timerCallback()
{
    map_.header.stamp = get_clock()->now();

    // Convert from updated probability map to map
    std::transform(probability_map_.begin(), probability_map_.end(), map_.data.begin(), [](double value){
        return logodds2prob(value) * 100;
    });
    map_pub_->publish(map_);
}

}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_mapping::MappingWithKnownPoses>("mapping_with_known_poses");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
