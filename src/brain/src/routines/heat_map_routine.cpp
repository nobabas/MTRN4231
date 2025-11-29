#include "brain/routines/heat_map_routine.hpp"
using namespace std::chrono_literals;

HeatMapRoutine::HeatMapRoutine(RobotInterface tools) : tools_(tools) {}

bool HeatMapRoutine::run() {
    auto logger = tools_.node->get_logger();
    RCLCPP_INFO(logger, "--- Starting Corner Heat Map ---");

    std::vector<std::vector<float>> corners = {
        {0.25, 0.20, 0.40, -3.14, 0.0, 1.57}, 
        {0.60, 0.20, 0.40, -3.14, 0.0, 1.57},
        {0.60, 0.50, 0.40, -3.14, 0.0, 1.57},
        {0.25, 0.50, 0.40, -3.14, 0.0, 1.57}
    };

    for(size_t i=0; i<corners.size(); i++) {
        std::vector<float> hover = corners[i];
        moveTo(hover);

        std::vector<float> dive = hover;
        dive[2] = 0.25f; // Fixed depth
        moveTo(dive);

        rclcpp::sleep_for(2s);
        RCLCPP_INFO(logger, "Corner %zu Moisture: %.2f", i+1, *(tools_.latest_moisture));

        moveTo(hover);
    }
    return true;
}

bool HeatMapRoutine::moveTo(const std::vector<float>& pose) {
    auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
    req->command = "pose";
    for(float p : pose) req->positions.push_back(p);
    return tools_.move_client->async_send_request(req).get()->success;
}