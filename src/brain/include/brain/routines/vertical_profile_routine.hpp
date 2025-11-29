#pragma once
#include <map>
#include "brain/robot_interface.hpp"
#include "interfaces/msg/marker_data.hpp"

class VerticalProfileRoutine {
public:
    explicit VerticalProfileRoutine(RobotInterface tools);
    bool run(const std::map<int, interfaces::msg::MarkerData>& markers);
private:
    RobotInterface tools_;
    bool moveLine(const std::vector<float>& pose);
};