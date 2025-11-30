#pragma once
#include <map>
#include <vector>
#include <string>
#include "brain/robot_interface.hpp"
#include "interfaces/msg/marker_data.hpp"

class TopographyRoutine {
public:
    explicit TopographyRoutine(RobotInterface tools);
    bool run(const std::map<int, interfaces::msg::MarkerData>& markers);
private:
    RobotInterface tools_;
    bool moveTo(const std::vector<float>& pose, const std::string& command);
};