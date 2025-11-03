// First run the vision node. The vision node send the /blue_markers_coords 
// This is then recieved by the moveit_planning_server. 
// Teensy is sending soil moisture as a topic /soil_moisture. 
// this is sent to brain, which shows whether it is in the soil. 

// Brain then sends command to movement as it goes more down until a threshold for soil value
// Waits 1 sec --> Then it keeps on going until soil value is reached 
// Then it loops again to the vision node to the next blue marker. If no new blue marker, it stops.

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("brain");
    RCLCPP_INFO(node->get_logger(), "Brain node started");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}