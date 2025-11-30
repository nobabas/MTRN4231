#pragma once
#include <vector>
#include <string>
#include "brain/robot_interface.hpp"

class HeatMapRoutine {
public:
    explicit HeatMapRoutine(RobotInterface tools);
    bool run(); // No arguments needed
private:
    RobotInterface tools_;
    bool moveTo(const std::vector<float>& pose, const std::string& command);
};