#include "brain/routines/heat_map_routine.hpp"
#include <memory>
#include <chrono>
#include <vector>

using namespace std::chrono_literals;

HeatMapRoutine::HeatMapRoutine(RobotInterface tools) : tools_(tools) {}

bool HeatMapRoutine::run() {
    auto logger = tools_.node->get_logger();
    RCLCPP_INFO(logger, "--- Starting Corner Heat Map ---");

    // --- 1. START AT HOME (JOINT) ---
    std::vector<float> home_joints = {-1.3f, 1.57f, -1.83f, -1.57f, 0.0f, 0.0f};
    if (!moveTo(home_joints, "joint")) {
        RCLCPP_ERROR(logger, "Failed to move Home. Aborting.");
        return false;
    }

    std::vector<std::vector<float>> corners = {
        {0.25, 0.20, 0.40, -3.14, 0.0, 1.57}, 
        {0.60, 0.20, 0.40, -3.14, 0.0, 1.57},
        {0.60, 0.50, 0.40, -3.14, 0.0, 1.57},
        {0.25, 0.50, 0.40, -3.14, 0.0, 1.57}
    };

    // --- EXECUTE ---
    for(size_t i=0; i<corners.size(); i++) {
        std::vector<float> hover = corners[i];
        
        // A. Travel to Corner (CARTESIAN)
        moveTo(hover, "cartesian");

        // B. Dip down (CARTESIAN)
        std::vector<float> dive = hover;
        dive[2] = 0.25f; 
        moveTo(dive, "cartesian");

        rclcpp::sleep_for(2s);
        RCLCPP_INFO(logger, "Corner %zu Moisture: %.2f", i+1, *(tools_.latest_moisture));

        // C. Retract (CARTESIAN)
        moveTo(hover, "cartesian");
    }

    // --- 2. END AT HOME (JOINT) ---
    moveTo(home_joints, "joint");
    RCLCPP_INFO(logger, "Heat Map Complete.");
    return true;
}

bool HeatMapRoutine::moveTo(const std::vector<float>& pose, const std::string& command) {
    auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
    req->command = command;
    for(float p : pose) req->positions.push_back(p);
    
    auto future_result = tools_.move_client->async_send_request(req);
    
    if (future_result.wait_for(5s) != std::future_status::ready) {
        return false;
    }
    return future_result.get()->success;
}