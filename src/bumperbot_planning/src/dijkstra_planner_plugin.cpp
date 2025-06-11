#include <queue>
#include <vector>
#include <algorithm>

#include "bumperbot_planning/dijkstra_planner_plugin.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

using std::placeholders::_1;

namespace bumperbot_planning
{
/*DijkstraPlanner::DijkstraPlanner()
    : Node("dijkstra_node")
{   
    // Initialize Buffer and Listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create QoS for map subscriber
    rclcpp::QoS map_qos(10);
    map_qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

    // Initialize Subscribers
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, 
                                                                    std::bind(&DijkstraPlanner::mapCallback, this, _1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("/goal_pose", 10,
                                                                        std::bind(&DijkstraPlanner::goalCallback, this, _1));

    // Initialize Publishers
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/dijkstra/path", 10);
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/dijkstra/visited_map", 10);
                                                                        
}*/

/*void DijkstraPlanner::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
{
    map_ = map;
    visited_map_.header.frame_id = map->header.frame_id;
    visited_map_.info = map->info;
    visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1);
}*/

/*void DijkstraPlanner::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    if(!map_){
        RCLCPP_ERROR(get_logger(), "No Map received!");
        return;
    }

    // Reset the visited map cells
    visited_map_.data = std::vector<int8_t>(visited_map_.info.height * visited_map_.info.width, -1);

    // Use the TF Listener and Buffer to get and store robot pose
    // Transformation from map to the robot base
    geometry_msgs::msg::TransformStamped map_to_base_tf;

    try
    {
        map_to_base_tf = tf_buffer_->lookupTransform(map_->header.frame_id, "base_footprint", tf2::TimePointZero);
    }
    catch(const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(get_logger(), "Could not find transform from map to base footprint");
        return;
    }

    // Transformed Stamaped to Pose Message
    geometry_msgs::msg::Pose map_to_base_pose;
    map_to_base_pose.position.x = map_to_base_tf.transform.translation.x;
    map_to_base_pose.position.y = map_to_base_tf.transform.translation.y;
    map_to_base_pose.orientation = map_to_base_tf.transform.rotation;

    // Call the plan function
    nav_msgs::msg::Path path = plan(map_to_base_pose, pose->pose);

    if(!path.poses.empty()){
        RCLCPP_INFO(get_logger(), "shortest path found");
        path_pub_->publish(path);
    }
    else{
        RCLCPP_WARN(get_logger(), "No Path found to the goal");
    }
}*/

void DijkstraPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name, 
        std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)  
{
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();
}

void DijkstraPlanner::cleanup()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "Cleaning up plugin " << name_.c_str() << " of type Dijkstra Planner");
}

void DijkstraPlanner::activate()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "Activating the plugin " << name_.c_str() << " of type Dijkstra Planner");   
}

void DijkstraPlanner::deactivate()
{
    RCLCPP_INFO_STREAM(node_->get_logger(), "Deactivating the plugin " << name_.c_str() << " of type Dijkstra Planner");   
}

nav_msgs::msg::Path DijkstraPlanner::createPlan(const geometry_msgs::msg::PoseStamped& start, const geometry_msgs::msg::PoseStamped& goal)
{
    // Define directions that we want the algorithm to explore. We will use 4 neighbors in this case
    std::vector<std::pair<int, int>> explore_directions = { {-1, 0}, {1, 0}, {0, -1}, {0, 1} };

    // Priority Queue. <Object Type, Storage Structure, How to store>
    std::priority_queue<GraphNode, std::vector<GraphNode>, std::greater<GraphNode>> pending_nodes;

    // Created a vector for visited nodes
    std::vector<GraphNode> visited_nodes;

    // Convert the start to Graph Node
    pending_nodes.push(worldToGrid(start.pose));
    GraphNode active_node;

    while(!pending_nodes.empty() && rclcpp::ok())
    {   
        // Pop the Node with lowest cost
        active_node = pending_nodes.top();
        pending_nodes.pop();

        if(worldToGrid(goal.pose) == active_node)
        {
            break;
        }

        // Explore the neighbors
        for(const auto& direction : explore_directions)
        {
            GraphNode new_node = active_node + direction;

            // Check if the Node is already visited, if the Node is valid, and if the Node is free or occupied
            if(std::find(visited_nodes.begin(), visited_nodes.end(), new_node) == visited_nodes.end() && 
                poseOnMap(new_node) && costmap_->getCost(new_node.x_, new_node.y_) < 99)
            {
                new_node.cost = active_node.cost + 1 + costmap_->getCost(new_node.x_, new_node.y_);
                new_node.prev = std::make_shared<GraphNode>(active_node);

                // Push the new node to priority queue
                pending_nodes.push(new_node);
                visited_nodes.push_back(new_node);
            }
        }

        // Update the visted Map
        //visited_map_.data.at(poseToCell(active_node)) = 10;
        //map_pub_->publish(visited_map_);

    }

    // Trace the Path back
    nav_msgs::msg::Path path;
    path.header.frame_id = global_frame_;
    while(active_node.prev && rclcpp::ok())
    {
        // List of Poses for the path
        geometry_msgs::msg::Pose last_pose;
        last_pose = gridtoWorld(active_node);

        geometry_msgs::msg::PoseStamped last_pose_stamped;
        last_pose_stamped.header.frame_id = global_frame_;
        last_pose_stamped.pose = last_pose;
        path.poses.push_back(last_pose_stamped);

        // Trace back the parents
        active_node = *active_node.prev;
    }

    // Reverse the path
    std::reverse(path.poses.begin(), path.poses.end());
    return path;
}

GraphNode DijkstraPlanner::worldToGrid(const geometry_msgs::msg::Pose& pose)
{
    int grid_x = static_cast<int>((pose.position.x - costmap_->getOriginX()) / costmap_->getResolution());
    int grid_y = static_cast<int>((pose.position.y - costmap_->getOriginY()) / costmap_->getResolution());
    return GraphNode(grid_x, grid_y);

}

geometry_msgs::msg::Pose DijkstraPlanner::gridtoWorld(const GraphNode& node)
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = node.x_ * costmap_->getResolution() + costmap_->getOriginX();
    pose.position.y = node.y_ * costmap_->getResolution() + costmap_->getOriginY();
    return pose;
}

bool DijkstraPlanner::poseOnMap(const GraphNode& node)
{
    return node.x_ >= 0 && node.x_ < static_cast<int>(costmap_->getSizeInCellsX()) && 
            node.y_ >= 0 && node.y_ < static_cast<int>(costmap_->getSizeInCellsY());
}

unsigned int DijkstraPlanner::poseToCell(const GraphNode& node)
{
    return node.y_ * costmap_->getSizeInCellsX() + node.x_;
}

}

/*int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<bumperbot_planning::DijkstraPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}*/


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bumperbot_planning::DijkstraPlanner, nav2_core::GlobalPlanner)