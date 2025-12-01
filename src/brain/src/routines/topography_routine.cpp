#include "brain/routines/topography_routine.hpp"
#include <chrono>
#include <memory>
#include <vector>

// --- CONTROL SWITCH ---
#define USE_VISION 0

using namespace std::chrono_literals;

TopographyRoutine::TopographyRoutine(RobotInterface tools) : tools_(tools) {}

bool TopographyRoutine::run(const std::map<int, interfaces::msg::MarkerData>& markers) {
    auto logger = tools_.node->get_logger();
    RCLCPP_INFO(logger, "--- Starting Topography Mapper ---");
    
    // Go Home First
    if (!moveTo({-1.3f, 1.57f, -1.83f, -1.57f, 0.0f, 0.0f}, "joint")) return false;

    std::vector<std::vector<float>> targets;

#if USE_VISION
    if (markers.empty()) {
        RCLCPP_WARN(logger, "[Vision] No markers to map.");
        return false;
    }
    for (const auto &pair : markers) {
        auto arr = pair.second.pose;
        targets.push_back({(float)arr[0], (float)arr[1], 0.40f, -3.14f, 0.0f, 1.57f});
    }
#else
    RCLCPP_WARN(logger, "[Test] Using Hardcoded Grid.");
    targets.push_back({0.30f, 0.0f, 0.40f, -3.14f, 0.0f, 0.0f});
#endif

    double contact_threshold = 1000.0; 

    for (const auto &hover : targets) {
        if(!moveTo(hover, "cartesian")) continue;

        std::vector<float> dive = hover;
        dive[2] = 0.15f; 
        
        auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
        req->command = "cartesian";
        for(float p : dive) req->positions.push_back(p);
        
        auto future = tools_.move_client->async_send_request(req);
        while(rclcpp::ok()) {
            if(future.wait_for(10ms) == std::future_status::ready) break;
            if(*(tools_.latest_moisture) > contact_threshold) {
                auto stop_req = std::make_shared<std_srvs::srv::Trigger::Request>();
                tools_.stop_client->async_send_request(stop_req).wait();
                RCLCPP_INFO(logger, "Contact Detected!");
                break;
            }
        }
        moveTo(hover, "cartesian");
        // Use getEndEffectorLink from moveitserver.cpp
        // Joint state publisher
    }
    
    moveTo({-1.3f, 1.57f, -1.83f, -1.57f, 0.0f, 0.0f}, "joint");
    return true;
}

bool TopographyRoutine::moveTo(const std::vector<float>& pose, const std::string& command) {
    auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
    req->command = command;
    for(float p : pose) req->positions.push_back(p);
    auto future = tools_.move_client->async_send_request(req);
    if (future.wait_for(5s) != std::future_status::ready) return false;
    return future.get()->success;
}