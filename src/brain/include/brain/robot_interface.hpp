#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "interfaces/srv/move_request.hpp"
#include "interfaces/srv/vision_cmd.hpp"

struct RobotInterface {
    // The node is useful for logging
    rclcpp::Node::SharedPtr node;
    
    // The clients needed to move and see
    rclcpp::Client<interfaces::srv::MoveRequest>::SharedPtr move_client;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client;
    rclcpp::Client<interfaces::srv::VisionCmd>::SharedPtr vision_client;
    
    // Pointer to shared moisture data
    const double* latest_moisture; 
};