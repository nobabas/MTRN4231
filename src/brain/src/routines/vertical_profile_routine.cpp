#include "brain/routines/vertical_profile_routine.hpp"
using namespace std::chrono_literals;

VerticalProfileRoutine::VerticalProfileRoutine(RobotInterface tools) : tools_(tools) {}

bool VerticalProfileRoutine::run(const std::map<int, interfaces::msg::MarkerData>& markers) {
    auto logger = tools_.node->get_logger();
    RCLCPP_INFO(logger, "--- Starting Vertical Profile ---");

    std::vector<float> depths = {0.30f, 0.25f, 0.20f};

    for (const auto &pair : markers) {
        auto p = pair.second.pose;
        std::vector<float> pose = {(float)p[0], (float)p[1], 0.40f, -3.14f, 0.0f, 1.57f};
        
        moveLine(pose); // Hover

        for (float z : depths) {
            std::vector<float> target = pose;
            target[2] = z;
            moveLine(target);
            rclcpp::sleep_for(2s); // Wait for reading
            RCLCPP_INFO(logger, "Depth %.2f | Moisture: %.2f", z, *(tools_.latest_moisture));
        }
        moveLine(pose); // Retract
    }
    return true;
}

bool VerticalProfileRoutine::moveLine(const std::vector<float>& pose) {
    auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
    req->command = "line"; // Use "line" for vertical precision
    for(float p : pose) req->positions.push_back(p);
    return tools_.move_client->async_send_request(req).get()->success;
}