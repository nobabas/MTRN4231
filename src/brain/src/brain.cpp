#include "brain/brain_node.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

BrainNode::BrainNode() : Node("brain_node")
{
    declare_parameter<double>("soil_threshold", 1000.0);
    soil_threshold_ = get_parameter("soil_threshold").as_double();

    RCLCPP_INFO(get_logger(), "Brain Node initialized.");

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    reentrant_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    vision_client_ = create_client<interfaces::srv::VisionCmd>("vision_srv", rmw_qos_profile_services_default, reentrant_group_);
    move_client_   = create_client<interfaces::srv::MoveRequest>("/moveit_path_plan", rmw_qos_profile_services_default, reentrant_group_);
    stop_client_   = create_client<std_srvs::srv::Trigger>("moveit_stop", rmw_qos_profile_services_default, reentrant_group_);

    brain_srv_ = create_service<interfaces::srv::BrainCmd>(
        "brain_srv",
        std::bind(&BrainNode::brainServiceCallback, this, _1, _2),
        rmw_qos_profile_services_default,
        reentrant_group_);

    rclcpp::QoS sticky_qos(rclcpp::KeepLast(1));
    sticky_qos.transient_local();

    marker_sub_ = create_subscription<interfaces::msg::Marker2DArray>(
        "/blue_markers_coords", sticky_qos, std::bind(&BrainNode::markerCallback, this, _1));

    moist_sub_ = create_subscription<std_msgs::msg::Float32>(
        "/soil_moisture", 10, std::bind(&BrainNode::moistureCallback, this, _1));
    
    temp_pub_ = create_publisher<std_msgs::msg::Float32>("/soil_temperature", 10);

    tools_.node = this;
    tools_.move_client = move_client_;
    tools_.stop_client = stop_client_;
    tools_.vision_client = vision_client_;
    tools_.latest_moisture = &latest_moisture_;
    tools_.latest_temperature = &latest_temperature_;
    tools_.latest_contact_height = &latest_contact_height_;

    soil_routine_       = std::make_shared<SoilRoutine>(tools_);
    topography_routine_ = std::make_shared<TopographyRoutine>(tools_);
    vertical_routine_   = std::make_shared<VerticalProfileRoutine>(tools_);
    heat_map_routine_   = std::make_shared<HeatMapRoutine>(tools_);
}

void BrainNode::brainServiceCallback(const std::shared_ptr<interfaces::srv::BrainCmd::Request> req,
                                     std::shared_ptr<interfaces::srv::BrainCmd::Response> res)
{
    RCLCPP_INFO(get_logger(), "Received command: '%s'", req->command.c_str());
    {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        marker_map_.clear();
    }

    RCLCPP_INFO(get_logger(), "Requesting vision");
    auto vision_req = std::make_shared<interfaces::srv::VisionCmd::Request>();
    vision_req->command = "scan";   

    auto vision_future = vision_client_->async_send_request(vision_req);
    
    // WAIT for tf_main to finish scanning (it will block here until it publishes data)
    // We give it 15 seconds to be safe (it usually takes ~3s)
    if (vision_future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
         RCLCPP_ERROR(get_logger(), "Vision Scan timed out!");
         res->success = false;
         return;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
    
    // The service has returned, which means tf_main has published the new markers 
    // and our markerCallback has already updated marker_map_.
    std::map<int, interfaces::msg::MarkerData> current_markers;
    {
        std::lock_guard<std::mutex> lock(marker_mutex_);
        current_markers = marker_map_; // Safe copy created here
    }
    
    RCLCPP_INFO(get_logger(), "Scan Finished. Found %zu markers.", current_markers.size());

    if (req->command == "soil_sampling") {
        // <--- FIXED: Use current_markers instead of marker_map_
        res->success = soil_routine_->run(soil_threshold_, current_markers); 
    }
    else if (req->command == "topography") {
        // <--- FIXED: Use current_markers instead of marker_map_
        res->success = topography_routine_->run(current_markers);
    }
    else if (req->command == "vertical") {
        // <--- FIXED: Use current_markers instead of marker_map_
        res->success = vertical_routine_->run(current_markers);
    }
    else if (req->command == "heat_map") {
        res->success = heat_map_routine_->run();
    }
    else {
        RCLCPP_WARN(get_logger(), "Unknown command.");
        res->success = false;
    }
}

void BrainNode::markerCallback(const interfaces::msg::Marker2DArray::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(marker_mutex_);
    
    // Threshold to consider a marker "new" (e.g., 5cm)
    double duplicate_radius = 0.04; 

    for (const auto &marker_2d : msg->markers) {
        float new_x = marker_2d.x;
        float new_y = marker_2d.y;

        // --- SPATIAL FILTERING ---
        bool is_duplicate = false;
        
        for (const auto &existing_pair : marker_map_) {
            auto existing_pose = existing_pair.second.pose;
            float ex_x = existing_pose[0];
            float ex_y = existing_pose[1];

            // Calculate Euclidean distance (2D)
            float dist = std::sqrt(std::pow(new_x - ex_x, 2) + std::pow(new_y - ex_y, 2));
            
            if (dist < duplicate_radius) {
                is_duplicate = true;
                // Optionally log debug info
                // RCLCPP_DEBUG(get_logger(), "Ignored duplicate marker at (%.2f, %.2f)", new_x, new_y);
                break; 
            }
        }

        if (is_duplicate) {
            continue; 
        }

        interfaces::msg::MarkerData full_marker;
        full_marker.id = marker_2d.id;
        // Z default = 0.35m
        full_marker.pose = {new_x, new_y, 0.35, -3.1415, 0.0, 0.0};
        
        marker_map_[static_cast<int>(full_marker.id)] = full_marker;
        
        RCLCPP_INFO(get_logger(), "Brain: Added Marker ID %.0f at [%.2f, %.2f]", 
                    full_marker.id, new_x, new_y);
    }
}

void BrainNode::moistureCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    latest_moisture_ = msg->data;
    latest_temperature_ = 35.0 - (latest_moisture_ / 50.0);

    if (latest_moisture_ > soil_threshold_) {
        try {
            // Lookup where the "tool0" frame is relative to "base_link"
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                "base_link", "tool0", tf2::TimePointZero);
            latest_contact_height_ = t.transform.translation.z;
            
        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(get_logger(), "Could not transform tool0 to base_link: %s", ex.what());
            latest_contact_height_ = 0.0;
        }
    }

    std_msgs::msg::Float32 temp_msg;
    temp_msg.data = latest_temperature_;
    if(temp_pub_) {
        temp_pub_->publish(temp_msg);
    }
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