#include "brain/routines/soil_routine.hpp"
#include <chrono>
#include <memory>
#include <vector>
#include <string>

// --- CONTROL SWITCH ---
#define USE_VISION 0

using namespace std::chrono_literals;

SoilRoutine::SoilRoutine(RobotInterface tools) : tools_(tools) {}

bool SoilRoutine::run(double soil_threshold, const std::map<int, interfaces::msg::MarkerData>& markers_input)
{
    auto logger = tools_.node->get_logger();
    RCLCPP_INFO(logger, "--- Starting Soil Sampling Routine ---");

    std::map<int, interfaces::msg::MarkerData> markers_to_visit;

#if USE_VISION
    if (markers_input.empty()) {
        RCLCPP_WARN(logger, "[Vision] No markers received. Aborting.");
        return false;
    }
    markers_to_visit = markers_input;
    RCLCPP_INFO(logger, "[Vision] Processing %zu markers.", markers_to_visit.size());
#else
    RCLCPP_WARN(logger, "[Test] Using HARDCODED markers.");
    interfaces::msg::MarkerData m1, m2, m3;
        
    // Marker 1 (Front Left)
        m1.id = 1; 
        m1.pose = {0.25, 0.62, 0.40, -3.1415, 0.0, 0.0};
        
        // Marker 2 (Front Right)
        m2.id = 2; 
        m2.pose = {0.60, 0.35, 0.40, -3.1415, 0.0, 0.0};
        
        // Marker 3 (Back Right)
        m3.id = 3; 
        m3.pose = {0.40, 0.25, 0.40, -3.1415, 0.0, 0.0};
        

        markers_to_visit[m1.id] = m1;
        markers_to_visit[m2.id] = m2;
        markers_to_visit[m3.id] = m3;
#endif

    // Go Home First
    if (!callMovementService("joint", {-1.3f, 1.57f, -1.83f, -1.57f, 0.0f, 0.0f})) return false;

    for (const auto &pair : markers_to_visit)
    {
        std::vector<float> current_pose(pair.second.pose.begin(), pair.second.pose.end());

        // Hover
        if (!callMovementService("cartesian", current_pose)) continue;

        // Descend
        std::vector<float> target = current_pose;
        target[2] = 0.25f; // Limit
        auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
        req->command = "cartesian"; 
        for(float p : target) req->positions.push_back(p);

        auto future = tools_.move_client->async_send_request(req);
        while(rclcpp::ok()) {
            if(future.wait_for(10ms) == std::future_status::ready) break;
            if(*(tools_.latest_moisture) < soil_threshold) {
                auto stop_req = std::make_shared<std_srvs::srv::Trigger::Request>();
                tools_.stop_client->async_send_request(stop_req).wait();
                rclcpp::sleep_for(1s);
                break;
            }
        }
        
        // Retract
        target[2] = (float)current_pose[2];
        callMovementService("cartesian", target);
    }
    
    callMovementService("joint", {-1.3f, 1.57f, -1.83f, -1.57f, 0.0f, 0.0f});
    return true;
}

bool SoilRoutine::callMovementService(const std::string &command, const std::vector<float> &positions)
{
    auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
    req->command = command;
    for (float p : positions) req->positions.push_back(p);
    if (!tools_.move_client->wait_for_service(std::chrono::seconds(1))) return false;
    return tools_.move_client->async_send_request(req).get()->success;
}