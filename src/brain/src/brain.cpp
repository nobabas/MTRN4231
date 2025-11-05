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
        declare_parameter("soil_threshold", 500);
        soil_threshold_ = get_parameter("soil_threshold").as_double();
        RCLCPP_INFO(get_logger(), "Brain Node started (soil_threshold = %.2f)", soil_threshold_);

        // -------------------------------
        // CLIENTS
        // -------------------------------
        vision_client_ = create_client<interfaces::srv::VisionCmd>("vision_srv");
        move_client_   = create_client<interfaces::srv::MoveRequest>("move_srv");

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
            });

        // -------------------------------
        // SUBSCRIPTIONS
        // -------------------------------
        marker_sub_ = create_subscription<interfaces::msg::MarkerData>(
            "/blue_markers_coords", 10,
            [this](const interfaces::msg::MarkerData::SharedPtr msg) {
                latest_marker_ = *msg;
                if (msg->pose.size() >= 3) {
                    RCLCPP_INFO(get_logger(), "Marker ID %.0f at [%.2f, %.2f, %.2f]",
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

private:
    // ----------------------------------------------------
    // MAIN ROUTINE
    // ----------------------------------------------------
    int runSoilSamplingRoutine() {
        RCLCPP_INFO(get_logger(), "Starting soil sampling routine...");

        // Step 1: Request marker coordinates
        auto marker = callVisionService("detect_marker");
        if (marker.pose.empty()) {
            RCLCPP_ERROR(get_logger(), "No marker pose returned from Vision node.");
            return 0;
        }

        // Step 2: Move to marker position (cartesian)
        std::vector<double> current_pose = marker.pose;
        if (!callMovementService("cartesian", current_pose)) {
            RCLCPP_ERROR(get_logger(), "Movement to marker failed.");
            return 0;
        }

        // Step 3: Lower probe incrementally until threshold reached
        /*
         * MOVE DOWN STRATEGY
         * ---------------------------------------------------------
         * The Brain tracks the current pose and updates the Z
         * coordinate directly. Each iteration lowers Z by 5 mm
         * (0.005 m) and sends a new MoveRequest in cartesian mode.
         *
         * This continues until soil_moisture ≥ soil_threshold_.
         * ---------------------------------------------------------
         */
        const double step_size = 0.005; // 5mm step
        while (latest_moisture_ < soil_threshold_) {
            current_pose[2] -= step_size;  // Lower Z
            RCLCPP_INFO(get_logger(),
                        "Moisture %.2f < %.2f → moving down to z=%.3f",
                        latest_moisture_, soil_threshold_, current_pose[2]);

            if (!callMovementService("cartesian", current_pose)) {
                RCLCPP_WARN(get_logger(), "Move down step failed, retrying...");
            }
            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        RCLCPP_INFO(get_logger(), "Soil threshold reached (%.2f ≥ %.2f)",
                    latest_moisture_, soil_threshold_);
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

    bool callMovementService(const std::string &command, const std::vector<double> &positions) {
        auto req = std::make_shared<interfaces::srv::MoveRequest::Request>();
        req->command = command;
        req->positions = positions;

        if (!move_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_WARN(get_logger(), "Movement service not available.");
            return false;
        }

        auto result = move_client_->async_send_request(req).get();
        return result->success;
    }

    // ----------------------------------------------------
    // VARIABLES & INTERFACES
    // ----------------------------------------------------
    double soil_threshold_;
    double latest_moisture_;
    interfaces::msg::MarkerData latest_marker_;

    rclcpp::Client<interfaces::srv::VisionCmd>::SharedPtr vision_client_;
    rclcpp::Client<interfaces::srv::MoveRequest>::SharedPtr move_client_;

    rclcpp::Service<interfaces::srv::BrainCmd>::SharedPtr brain_srv_;

    rclcpp::Subscription<interfaces::msg::MarkerData>::SharedPtr marker_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr moist_sub_;
};

// ----------------------------------------------------
// MAIN
// ----------------------------------------------------
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BrainNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

