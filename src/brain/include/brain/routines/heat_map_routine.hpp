#pragma once
#include "brain/robot_interface.hpp"

class HeatMapRoutine {
public:
    explicit HeatMapRoutine(RobotInterface tools);
    bool run();
private:
    RobotInterface tools_;
    bool moveTo(const std::vector<float>& pose);
};