#include "brain/routines/soil_routine.hpp"
#include <chrono>
#include <memory>
#include <vector>
#include <string>

// Set to 0 to use hardcoded test points instead of vision
#define USE_VISION 1

using namespace std::chrono_literals;

SoilRoutine::SoilRoutine(RobotInterface tools) : tools_(tools) {}

bool SoilRoutine::run(double soil_threshold, const std::map<int, interfaces::msg::MarkerData>& markers_input)
{
    auto logger = tools_.node->get_logger();
    RCLCPP_INFO(logger, "--- Starting Soil Sampling Routine ---");

    std::map<int, interfaces::msg::MarkerData> markers_to_visit;

#if USE_VISION
    if (markers_input.empty()) {
        RCLCPP_WARN(logger, "No markers received. Aborting routine.");
        return false;
    }
    markers_to_visit = markers_input;
    RCLCPP_INFO(logger, "Using %zu Vision Markers.", markers_to_visit.size());
#else
    RCLCPP_INFO(logger, "Using HARDCODED Markers (Test Mode).");
    interfaces::msg::MarkerData m1, m2;
    m1.id = 1; m1.pose = {0.205, 0.62, 0.5, -3.1415, 0, 0};
    m2.id = 2; m2.pose = {0.60, 0.35, 0.5, -3.1415, 0, 0};
    markers_to_visit[m1.id] = m1;
    markers_to_visit[m2.id] = m2;
#endif

    for (const auto &pair : markers_to_visit)
    {
        const int marker_id = pair.first;
        const auto &marker = pair.second;

        RCLCPP_INFO(logger, ">>> Moving to Marker %d", marker_id);

        std::vector<float> current_pose(marker.pose.begin(), marker.pose.end());

        // 1. Go Home First
        std::vector<float> home_position = {-1.3, 1.57, -1.83, -1.57, 0.0, 0.0};
        if (!callMovementService("joint", home_position)) continue;

        // 2. Move to Marker Hover Position
        if (!callMovementService("cartesian", current_pose)) continue;

        // 3. Descend
        double safe_z = current_pose[2];
        std::vector<float> target = current_pose;
        target[2] = 0.25; // Limit

        auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
        req->command = "cartesian"; 
        for(float p : target) req->positions.push_back((double)p);

        RCLCPP_INFO(logger, "Descending... (Stop if moisture < %.2f)", soil_threshold);
        auto future = tools_.move_client->async_send_request(req);

        while(rclcpp::ok()) {
            if(future.wait_for(10ms) == std::future_status::ready) {
                RCLCPP_INFO(logger, "Reached bottom limit.");
                break; 
            }

            double current_moist = *(tools_.latest_moisture);
            if(current_moist < soil_threshold) {
                RCLCPP_WARN(logger, "!!! MOISTURE DETECTED (%.2f) !!! STOPPING.", current_moist);
                auto stop_req = std::make_shared<std_srvs::srv::Trigger::Request>();
                
                // FIX: Changed '->wait()' to '.wait()' because async_send_request returns an object, not a pointer
                tools_.stop_client->async_send_request(stop_req).wait();
                
                rclcpp::sleep_for(1s);
                break;
            }
        }

        // 4. Retract
        target[2] = (float)safe_z;
        callMovementService("cartesian", target);
    }
    
    callMovementService("joint", {-1.3, 1.57, -1.83, -1.57, 0.0, 0.0});
    return true;
}

// FIX: This implementation must match the header exactly
bool SoilRoutine::callMovementService(const std::string &command, const std::vector<float> &positions)
{
    auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
    req->command = command;
    for (float p : positions) req->positions.push_back((double)p);

    if (!tools_.move_client->wait_for_service(1s)) {
        RCLCPP_WARN(tools_.node->get_logger(), "Move service busy/offline.");
        return false;
    }
    auto result = tools_.move_client->async_send_request(req).get();
    return result->success;
}