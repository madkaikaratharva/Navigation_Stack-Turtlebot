#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "twist_mux_msgs/action/joy_turbo.hpp"

// Visualize safety markers in rviz
#include "visualization_msgs/msg/marker_array.hpp"



using namespace std::placeholders;
using namespace std::chrono_literals;

enum State
{
    FREE_STATE=0,
    WARNING_STATE=1,
    DANGER_STATE=2
};

class SafetyStop : public rclcpp::Node
{
private:
    bool is_first_msg_;
    double danger_distance_;
    double warning_distance_;
    visualization_msgs::msg::MarkerArray zones_;

    State state_, prev_state_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_; 
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_stop_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr zones_pub_;

    // Action Clients
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr decrease_speed_client_;
    rclcpp_action::Client<twist_mux_msgs::action::JoyTurbo>::SharedPtr increase_speed_client_;

public:
    SafetyStop()
        : Node("safety_stop_node")
        , is_first_msg_(true)
        , state_(State::FREE_STATE)
        , prev_state_(State::FREE_STATE)
    {
        // Declaring Parameters
        declare_parameter<double>("danger_distance", 0.2);
        declare_parameter<double>("warning_distance", 0.6);
        declare_parameter<std::string>("scan_topic", "scan");
        declare_parameter<std::string>("safety_stop_topic", "safety_stop");

        // Get Parameter Values
        danger_distance_ = get_parameter("danger_distance").as_double();
        warning_distance_ = get_parameter("warning_distance").as_double();
        std::string scan_topic = get_parameter("scan_topic").as_string();
        std::string safe_stop_topic = get_parameter("safety_stop_topic").as_string();

        // Initialize Publishers and Subscribers
        laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(scan_topic, 10, 
                                                                        std::bind(&SafetyStop::laserCallback, this, _1));
        safety_stop_pub_ = create_publisher<std_msgs::msg::Bool>(safe_stop_topic, 10);
        zones_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/zones", 10);

        // Initialize Clients
        decrease_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_decrease");
        increase_speed_client_ = rclcpp_action::create_client<twist_mux_msgs::action::JoyTurbo>(this, "joy_turbo_increase");


        // Checks if the action server is active
        while(!decrease_speed_client_->wait_for_action_server(1s) && rclcpp::ok())
        {
            RCLCPP_WARN(get_logger(), "Action Joy turbo decrease not available! Waiting......");
            std::this_thread::sleep_for(2s);
        }

        while(!increase_speed_client_->wait_for_action_server(1s) && rclcpp::ok())
        {
            RCLCPP_WARN(get_logger(), "Action Joy turbo increase not available! Waiting......");
            std::this_thread::sleep_for(2s);
        }

        // Static Attributes of the Marker
        visualization_msgs::msg::Marker warning_zone;
        warning_zone.id = 0;
        warning_zone.action = visualization_msgs::msg::Marker::ADD; // Tells Rviz to visualize this marker
        warning_zone.type = visualization_msgs::msg::Marker::CYLINDER;
        warning_zone.scale.z = 0.001;
        warning_zone.scale.x = warning_distance_ * 2;   // Diameter of the display cylinder
        warning_zone.scale.y = warning_distance_ * 2;   // Diameter of the display cylinder
        warning_zone.color.r = 1.0;     // Red Color
        warning_zone.color.g = 0.98;    // Green Color
        warning_zone.color.b = 0.0;     // Blue Color
        warning_zone.color.a = 0.5;     // Alpha
        zones_.markers.push_back(warning_zone);

        visualization_msgs::msg::Marker danger_zone;
        danger_zone.id = 1;
        danger_zone.action = visualization_msgs::msg::Marker::ADD; // Tells Rviz to visualize this marker
        danger_zone.type = visualization_msgs::msg::Marker::CYLINDER;
        danger_zone.scale.z = 0.001;
        danger_zone.scale.x = danger_distance_ * 2;   // Diameter of the display cylinder
        danger_zone.scale.y = danger_distance_ * 2;   // Diameter of the display cylinder
        danger_zone.color.r = 1.0;     // Red Color
        danger_zone.color.g = 0.0;    // Green Color
        danger_zone.color.b = 0.0;     // Blue Color
        danger_zone.color.a = 0.5;     // Alpha
        zones_.markers.push_back(danger_zone);

    }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan& msg)
    {
        state_ = State::FREE_STATE;

        // Checks if the ray is valid and if the returned value is less than 20cm (0.2 m)
        for(const auto& range : msg.ranges)
        {
            if(!std::isinf(range) && range <= warning_distance_)
            {
                state_ = State::WARNING_STATE;

                if(range <= danger_distance_)
                {
                    state_ = State::DANGER_STATE;
                    break;
                }

            }
        }

        if(state_ != prev_state_)
        {
            std_msgs::msg::Bool is_safety_stop;
            if(state_ == State::WARNING_STATE)
            {
                is_safety_stop.data = false;
                decrease_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
                zones_.markers.at(0).color.a = 1.0;
                zones_.markers.at(1).color.a = 0.5;
            }
            else if(state_ == State::DANGER_STATE)
            {
                is_safety_stop.data = true;
                zones_.markers.at(0).color.a = 1.0;
                zones_.markers.at(1).color.a = 1.0;
            } 
            else if(state_ == State::FREE_STATE)
            {
                is_safety_stop.data = false;
                zones_.markers.at(0).color.a = 0.5;
                zones_.markers.at(1).color.a = 0.5;
                increase_speed_client_->async_send_goal(twist_mux_msgs::action::JoyTurbo::Goal());
            }

            prev_state_ = state_;

            safety_stop_pub_->publish(is_safety_stop);

        }

        if(is_first_msg_)
        {
            for(visualization_msgs::msg::Marker& zone: zones_.markers)
            {
                zone.header.frame_id = msg.header.frame_id;
            }
            is_first_msg_ = false;
        }

        zones_pub_->publish(zones_);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SafetyStop>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}