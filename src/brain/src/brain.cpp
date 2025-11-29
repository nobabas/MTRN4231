#include "brain/brain_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

BrainNode::BrainNode() : Node("brain_node")
{
    // 1. Parameters
    declare_parameter<double>("soil_threshold", 1000.0);
    soil_threshold_ = get_parameter("soil_threshold").as_double();
    RCLCPP_INFO(get_logger(), "Brain Node initialized (Threshold: %.2f)", soil_threshold_);

    reentrant_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 2. Setup Clients
    vision_client_ = create_client<interfaces::srv::VisionCmd>("vision_srv", rmw_qos_profile_services_default, reentrant_group_);
    move_client_   = create_client<interfaces::srv::MoveRequest>("/moveit_path_plan", rmw_qos_profile_services_default, reentrant_group_);
    stop_client_   = create_client<std_srvs::srv::Trigger>("moveit_stop", rmw_qos_profile_services_default, reentrant_group_);

    // 3. Setup Service (The Switchboard)
    brain_srv_ = create_service<interfaces::srv::BrainCmd>(
        "brain_srv",
        std::bind(&BrainNode::brainServiceCallback, this, _1, _2),
        rmw_qos_profile_services_default,
        reentrant_group_);

    // 4. Setup Subscriptions
    rclcpp::QoS sticky_qos(rclcpp::KeepLast(1));
    sticky_qos.transient_local();

    marker_sub_ = create_subscription<interfaces::msg::Marker2DArray>(
        "/blue_markers_coords", sticky_qos, std::bind(&BrainNode::markerCallback, this, _1));

    moist_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/soil_moisture", 10, std::bind(&BrainNode::moistureCallback, this, _1));

    // 5. Pack the Toolbox
    tools_.node = shared_from_this();
    tools_.move_client = move_client_;
    tools_.stop_client = stop_client_;
    tools_.vision_client = vision_client_;
    tools_.latest_moisture = &latest_moisture_;

    // 6. Initialize Routines
    soil_routine_       = std::make_shared<SoilRoutine>(tools_);
    topography_routine_ = std::make_shared<TopographyRoutine>(tools_);
    vertical_routine_   = std::make_shared<VerticalProfileRoutine>(tools_);
    heat_map_routine_   = std::make_shared<HeatMapRoutine>(tools_);
}

void BrainNode::brainServiceCallback(const std::shared_ptr<interfaces::srv::BrainCmd::Request> req,
                                     std::shared_ptr<interfaces::srv::BrainCmd::Response> res)
{
    RCLCPP_INFO(get_logger(), "Received command: '%s'", req->command.c_str());

    if (req->command == "soil_sampling") {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        res->success = soil_routine_->run(soil_threshold_, marker_map_);
    }
    else if (req->command == "topography") {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        res->success = topography_routine_->run(marker_map_);
    }
    else if (req->command == "vertical") {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        res->success = vertical_routine_->run(marker_map_);
    }
    else if (req->command == "heat_map") {
        res->success = heat_map_routine_->run(); // Heat map doesn't need markers
    }
    else {
        RCLCPP_WARN(get_logger(), "Unknown command.");
        res->success = false;
    }
}

void BrainNode::markerCallback(const interfaces::msg::Marker2DArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(marker_mutex_);
    for (const auto &marker_2d : msg->markers) {
        interfaces::msg::MarkerData full_marker;
        full_marker.id = marker_2d.id;
        full_marker.pose = {marker_2d.x, marker_2d.y, 0.35, -3.1415, 0.0, 1.57}; // Add defaults
        marker_map_[static_cast<int>(full_marker.id)] = full_marker;
    }
    RCLCPP_INFO_ONCE(get_logger(), "Brain: Markers received.");
}

void BrainNode::moistureCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_moisture_ = msg->data;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BrainNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}