#pragma once

#include <map>
#include <mutex>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "interfaces/msg/marker2_d_array.hpp"
#include "interfaces/srv/brain_cmd.hpp"

// Include shared tools
#include "brain/robot_interface.hpp"

// Include Routines
#include "brain/routines/soil_routine.hpp"
#include "brain/routines/topography_routine.hpp"
#include "brain/routines/vertical_profile_routine.hpp"
#include "brain/routines/heat_map_routine.hpp"

class BrainNode : public rclcpp::Node {
public:
    BrainNode();
    
    void init(); 

private:
    // --- Callbacks ---
    void brainServiceCallback(const std::shared_ptr<interfaces::srv::BrainCmd::Request> req,
                              std::shared_ptr<interfaces::srv::BrainCmd::Response> res);
    
    void markerCallback(const interfaces::msg::Marker2DArray::SharedPtr msg);
    void moistureCallback(const std_msgs::msg::Float32::SharedPtr msg);

    // --- Data ---
    double soil_threshold_;
    double latest_moisture_ = 0.0;
    
    std::map<int, interfaces::msg::MarkerData> marker_map_;
    std::mutex marker_mutex_;

    // --- ROS Interfaces ---
    rclcpp::CallbackGroup::SharedPtr reentrant_group_;
    
    rclcpp::Client<interfaces::srv::VisionCmd>::SharedPtr vision_client_;
    rclcpp::Client<interfaces::srv::MoveRequest>::SharedPtr move_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
    
    rclcpp::Service<interfaces::srv::BrainCmd>::SharedPtr brain_srv_;
    rclcpp::Subscription<interfaces::msg::Marker2DArray>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr moist_sub_;

    // --- Routines & Tools ---
    RobotInterface tools_;
    
    std::shared_ptr<SoilRoutine> soil_routine_;
    std::shared_ptr<TopographyRoutine> topography_routine_;
    std::shared_ptr<VerticalProfileRoutine> vertical_routine_;
    std::shared_ptr<HeatMapRoutine> heat_map_routine_;
};