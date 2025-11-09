/*
 * ===========================================================
 *  BRAIN NODE – Soil Sampling Robot (Brain-Manages-Motion)
 * ===========================================================
 *  Coordinates Vision, Movement, and Moisture subsystems.
 *  Receives blue marker detections from the vision node,
 *  requests soil moisture from the Teensy via /soil_moisture,
 *  and commands the movement system directly.
 *
 *  Interfaces:
 *   - VisionCmd.srv        : Brain → Vision Node (detects blue markers)
 *   - MoveRequest.srv      : Brain → Movement Node (execute motion)
 *   - BrainCmd.srv         : External → Brain (start routines)
 *   - /soil_moisture (topic): Teensy → Brain (current moisture)
 *   - MarkerData.msg       : Published by Vision Node
 *
 *  Routine Flow:
 *   1. Brain requests marker pose from Vision Node
 *   2. Sends MoveRequest to go to that pose (cartesian)
 *   3. Monitors /soil_moisture readings
 *   4. Lowers probe incrementally in Z until threshold reached
 *
 *  Parameter:
 *   soil_threshold (default: 0.5)
 * ===========================================================
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "std_msgs/msg/float32.hpp"

#include "interfaces/msg/marker_data.hpp"
#include "interfaces/srv/vision_cmd.hpp"
#include "interfaces/srv/move_request.hpp"
#include "interfaces/srv/brain_cmd.hpp"

class BrainNode : public rclcpp::Node {
public:
    BrainNode() : Node("brain_node") {

        // -------------------------------
        // PARAMETERS
        // -------------------------------
        declare_parameter<double>("soil_threshold", 690.0);
        soil_threshold_ = get_parameter("soil_threshold").as_double();
        RCLCPP_INFO(get_logger(), "Brain Node started (soil_threshold = %.2f)", soil_threshold_);

        reentrant_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // -------------------------------
        // CLIENTS
        // -------------------------------
        vision_client_ = create_client<interfaces::srv::VisionCmd>("vision_srv", rmw_qos_profile_services_default, reentrant_group_) ;
        move_client_   = create_client<interfaces::srv::MoveRequest>("/moveit_path_plan", rmw_qos_profile_services_default, reentrant_group_);

        // -------------------------------
        // SERVICE (External Command)
        // -------------------------------
        brain_srv_ = create_service<interfaces::srv::BrainCmd>(
            "brain_srv",
            [this](const std::shared_ptr<interfaces::srv::BrainCmd::Request> req,
                   std::shared_ptr<interfaces::srv::BrainCmd::Response> res) {
                RCLCPP_INFO(get_logger(), "Brain received command: %s", req->command.c_str());
                if (req->command == "soil_sampling") {
                    res->success = (runSoilSamplingRoutine() == 1);
                } else {
                    RCLCPP_WARN(get_logger(), "Unknown brain command: %s", req->command.c_str());
                    res->success = false;
                }
            },
            rmw_qos_profile_services_default,
            reentrant_group_
        );

        // -------------------------------
        // SUBSCRIPTIONS
        // -------------------------------
        marker_sub_ = create_subscription<interfaces::msg::MarkerData>(
            "/blue_markers_coords", 10,
            [this](const interfaces::msg::MarkerData::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(marker_mutex_);
                marker_map_[static_cast<int>(msg->id)] = *msg;

                if (msg->pose.size() >= 3) {
                    RCLCPP_INFO(get_logger(), "Brain: Marker ID %.0f at [%.2f, %.2f, %.2f]",
                                msg->id, msg->pose[0], msg->pose[1], msg->pose[2]);
                } else {
                    RCLCPP_WARN(get_logger(), "MarkerData.pose malformed or empty.");
                }
            });

        moist_sub_ = create_subscription<std_msgs::msg::Float32>(
            "/soil_moisture", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                latest_moisture_ = msg->data;
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Current soil moisture: %.2f", latest_moisture_);
            });

        latest_moisture_ = 0.0;
    }


    // ----------------------------------------------------
    // MAIN ROUTINE
    // ----------------------------------------------------
    // Need to put call vision service in here once it is done 
    int runSoilSamplingRoutine() {
        RCLCPP_INFO(get_logger(), "Starting soil sampling routine...");

        // --------------------------------------------------------
        // Step 1: Define hardcoded markers (temporary)
        // --------------------------------------------------------
        std::map<int, interfaces::msg::MarkerData> markers_copy;

        {
            interfaces::msg::MarkerData m1, m2, m3;

            m1.id = 1;
            m1.pose = {0.60, -0.35, 0.25, -1.57, 1.57, 0.0};

            m2.id = 2;
            m2.pose = {0.55, -0.30, 0.25, -1.57, 1.57, 0.0};

            m3.id = 3;
            m3.pose = {0.50, -0.25, 0.25, -1.57, 1.57, 0.0};

            markers_copy[m1.id] = m1;
            markers_copy[m2.id] = m2;
            markers_copy[m3.id] = m3;
        }

        RCLCPP_INFO(get_logger(), "Loaded %zu hardcoded marker(s).", markers_copy.size());

        // --------------------------------------------------------
        // Step 2: Iterate through each marker
        // --------------------------------------------------------
        for (const auto &pair : markers_copy) {
            const int marker_id = pair.first;
            const auto &marker = pair.second;

            if (marker.pose.size() < 6) {
                RCLCPP_WARN(get_logger(), "Marker %d has invalid pose size (%zu). Skipping.",
                            marker_id, marker.pose.size());
                continue;
            }

            RCLCPP_INFO(get_logger(),
                        "\n=============================\n"
                        " Moving to Marker %d\n"
                        "=============================",
                        marker_id);

            std::vector<float> current_pose(marker.pose.begin(), marker.pose.end());

            // Wait until movement service is available
            while (!move_client_->wait_for_service(std::chrono::seconds(1))) {
                RCLCPP_WARN(get_logger(), "Waiting for MoveIt service to be available...");
            }

            // --- Move to marker ---
            if (!callMovementService("cartesian", current_pose)) {
                RCLCPP_ERROR(get_logger(), "Movement to marker %d failed. Skipping to next.", marker_id);
                continue;
            }

            RCLCPP_INFO(get_logger(), "Arrived at marker %d. Beginning soil sampling.", marker_id);

            // ----------------------------------------------------
            // Step 3: Lower probe until threshold reached
            // ----------------------------------------------------
            const double step_size = 0.005;  // 5 mm down each iteration
            const double min_z_limit = -0.05; // safety limit below which we stop
            double initial_z = current_pose[2];
            latest_moisture_ = 0.0; // reset each time

            while (latest_moisture_ < soil_threshold_) {
                current_pose[2] -= step_size;

                if (current_pose[2] < min_z_limit) {
                    RCLCPP_ERROR(get_logger(),
                                "Safety stop: Z below %.3f while sampling marker %d. Aborting sampling for this marker.",
                                min_z_limit, marker_id);
                    break;
                }

                RCLCPP_INFO(get_logger(),
                            "Marker %d → Moisture %.2f < %.2f → lowering probe to z = %.3f",
                            marker_id, latest_moisture_, soil_threshold_, current_pose[2]);

                if (!callMovementService("cartesian", current_pose)) {
                    RCLCPP_WARN(get_logger(), "Move down step failed (marker %d). Retrying...", marker_id);
                }

                rclcpp::sleep_for(std::chrono::seconds(1));
            }

            RCLCPP_INFO(get_logger(),
                        "Marker %d sampling complete (%.2f ≥ %.2f).",
                        marker_id, latest_moisture_, soil_threshold_);

            // ----------------------------------------------------
            // Step 4: Retract probe back to safe height
            // ----------------------------------------------------
            // Should do "line"
            current_pose[2] = initial_z;
            if (!callMovementService("cartesian", current_pose)) {
                RCLCPP_WARN(get_logger(), "Failed to retract probe after sampling marker %d.", marker_id);
            }

            RCLCPP_INFO(get_logger(), "Probe retracted to safe Z for marker %d.", marker_id);
        }

        // --------------------------------------------------------
        // Step 5: Routine complete
        // --------------------------------------------------------
        RCLCPP_INFO(get_logger(), "✅ Soil sampling routine completed for all markers.");
        return 1;
    }


    // ----------------------------------------------------
    // SERVICE HELPERS
    // ----------------------------------------------------
    interfaces::msg::MarkerData callVisionService(const std::string &command) {
        auto req = std::make_shared<interfaces::srv::VisionCmd::Request>();
        req->command = command;

        if (!vision_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(get_logger(), "Vision service not available.");
            return interfaces::msg::MarkerData();
        }

        auto future = vision_client_->async_send_request(req);
        auto result = future.get();
        return result->marker_data;
    }

    bool callMovementService(const std::string &command, const std::vector<float> &positions) {
        RCLCPP_WARN(get_logger(), "Movement service called.");
        
        auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
        req->command = command;

        RCLCPP_WARN(get_logger(), "Movement service called.2");

        // Convert float → double
        req->positions.reserve(positions.size());
        for (float p : positions)
            req->positions.push_back(static_cast<double>(p));
             RCLCPP_WARN(get_logger(), "Movement service called.3");
        

        if (!move_client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "Movement service not available.");
            return false;
        }

        auto result = move_client_->async_send_request(req).get();
        RCLCPP_WARN(get_logger(), "Movement service called.4");
        return result->success;
    }


    // ----------------------------------------------------
    // VARIABLES & INTERFACES
    // ----------------------------------------------------
    double soil_threshold_;
    double latest_moisture_;

    std::map<int, interfaces::msg::MarkerData> marker_map_;
    std::mutex marker_mutex_;

    interfaces::msg::MarkerData latest_marker_;

    rclcpp::Client<interfaces::srv::VisionCmd>::SharedPtr vision_client_;
    rclcpp::Client<interfaces::srv::MoveRequest>::SharedPtr move_client_;

    rclcpp::Service<interfaces::srv::BrainCmd>::SharedPtr brain_srv_;
    rclcpp::CallbackGroup::SharedPtr reentrant_group_;

    rclcpp::Subscription<interfaces::msg::MarkerData>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr moist_sub_;
};

// ----------------------------------------------------
// MAIN
// ----------------------------------------------------
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BrainNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}

