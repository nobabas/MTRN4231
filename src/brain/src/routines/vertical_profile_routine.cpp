#include "brain/routines/vertical_profile_routine.hpp"
#include <chrono>
#include <memory>
#include <vector>

// --- CONTROL SWITCH ---
#define USE_VISION 0

using namespace std::chrono_literals;

VerticalProfileRoutine::VerticalProfileRoutine(RobotInterface tools) : tools_(tools) {}

bool VerticalProfileRoutine::run(const std::map<int, interfaces::msg::MarkerData>& markers) {
    auto logger = tools_.node->get_logger();
    RCLCPP_INFO(logger, "--- Starting Vertical Profile ---");

    // Go Home First
    if (!move({-1.3f, 1.57f, -1.83f, -1.57f, 0.0f, 0.0f}, "joint")) return false;

    std::vector<std::vector<float>> targets;

#if USE_VISION
    if (markers.empty()) {
        RCLCPP_WARN(logger, "[Vision] No markers found.");
        return false;
    }
    for (const auto& pair : markers) {
        auto arr = pair.second.pose;
        targets.push_back({(float)arr[0], (float)arr[1], 0.40f, -3.14f, 0.0f, 1.57f});
    }
#else
    RCLCPP_WARN(logger, "[Test] Using Hardcoded Point.");
    targets.push_back({0.45f, 0.0f, 0.40f, -3.14f, 0.0f, 1.57f});
#endif

    std::vector<float> depths = {0.30f, 0.25f, 0.20f};

    for (const auto &pose : targets) {
        move(pose, "cartesian"); 
        for (float z : depths) {
            std::vector<float> target = pose;
            target[2] = z;
            move(target, "cartesian");
            rclcpp::sleep_for(2s); 
            RCLCPP_INFO(logger, "Depth %.2f | Moisture: %.2f", z, *(tools_.latest_moisture));
        }
        move(pose, "cartesian"); 
    }

    move({-1.3f, 1.57f, -1.83f, -1.57f, 0.0f, 0.0f}, "joint");
    return true;
}

bool VerticalProfileRoutine::move(const std::vector<float>& pose, const std::string& command) {
    auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
    req->command = command; 
    for(float p : pose) req->positions.push_back(p);
    auto future = tools_.move_client->async_send_request(req);
    if(future.wait_for(5s) != std::future_status::ready) return false;
    return future.get()->success;
}
