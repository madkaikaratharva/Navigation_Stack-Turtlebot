#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

// Contains the base class from which all the planners need to inherit
#include "nav2_core/global_planner.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace bumperbot_planning
{
struct GraphNode
{
    int x_;
    int y_;
    int cost;
    double heurestic;

    // Points to its parent
    std::shared_ptr<GraphNode> prev;

    // Default Constructor
    GraphNode()
        : x_(0), y_(0), cost(0), heurestic(0.0)
    {}

    // Parameterized Constructor
    GraphNode(int x, int y)
        : x_(x), y_(y), cost(0), heurestic(0.0)
    {}

    // Operator overloading
    bool operator>(const GraphNode& other) const
    {
        return cost + heurestic > other.cost + other.heurestic;
    }

    bool operator==(const GraphNode& other) const
    {
        return x_ == other.x_ && y_ == other.y_;
    }

    GraphNode operator+(std::pair<int, int> const& other)
    {
        GraphNode res(x_ + other.first, y_ + other.second);
        return res;
    }

};    

class AStarPlanner : public nav2_core::GlobalPlanner
{
private:
    // subscriber to receive map messages
    //rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    // subscriber to recieve goal pose
    //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    // publisher to publish path
    //rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    // publish the visited grids
    //rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    // Pointer to the grid map
    //nav_msgs::msg::OccupancyGrid::SharedPtr map_;
    // variable to store the map of visted grids
    //nav_msgs::msg::OccupancyGrid visited_map_;
    //
    //std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    //std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // Transform Buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // Lifecycle node from NAV2 planner
    nav2_util::LifecycleNode::SharedPtr node_;

    // Costmap2D from NAV2
    nav2_costmap_2d::Costmap2D* costmap_;

    // Global Frame Id and Algorithm name
    std::string global_frame_;
    std::string name_;

public:
    AStarPlanner() = default;
    ~AStarPlanner() = default;

    // Redefine the functions in the nav2_core::GlobalPlanner Base Class
    void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, 
                    std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal) override;

private:
    //void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);
    //void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    //nav_msgs::msg::Path plan(const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& goal);

    GraphNode worldToGrid(const geometry_msgs::msg::Pose& pose);
    geometry_msgs::msg::Pose gridtoWorld(const GraphNode& node);
    bool poseOnMap(const GraphNode& node);
    unsigned int poseToCell(const GraphNode& node);
    float getHeurestic(const GraphNode& node, const GraphNode& goal_node);

    
};  
}