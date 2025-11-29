#include "brain/routines/topography_routine.hpp"
using namespace std::chrono_literals;

TopographyRoutine::TopographyRoutine(RobotInterface tools) : tools_(tools) {}

bool TopographyRoutine::run(const std::map<int, interfaces::msg::MarkerData>& markers) {
    auto logger = tools_.node->get_logger();
    RCLCPP_INFO(logger, "--- Starting Topography Mapper ---");
    
    double contact_threshold = 200.0; // Adjust for your sensor

    for (const auto &pair : markers) {
        auto pose_vec = pair.second.pose;
        std::vector<float> hover = { (float)pose_vec[0], (float)pose_vec[1], 0.40f, -3.14f, 0.0f, 1.57f };

        // Hover
        if(!moveTo(hover)) continue;

        // Slow Descent
        std::vector<float> dive = hover;
        dive[2] = 0.15f; 
        
        auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
        req->command = "line";
        for(float p : dive) req->positions.push_back(p);
        auto future = tools_.move_client->async_send_request(req);

        while(rclcpp::ok()) {
            if(future.wait_for(10ms) == std::future_status::ready) break;

            if(*(tools_.latest_moisture) > contact_threshold) {
                auto stop_req = std::make_shared<std_srvs::srv::Trigger::Request>();
                tools_.stop_client->async_send_request(stop_req)->wait();
                RCLCPP_INFO(logger, "Surface found at Marker %.0f", pair.first);
                break;
            }
        }
        // Retract
        moveTo(hover);
    }
    return true;
}

bool TopographyRoutine::moveTo(const std::vector<float>& pose) {
    auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
    req->command = "pose";
    for(float p : pose) req->positions.push_back(p);
    return tools_.move_client->async_send_request(req).get()->success;
}