#pragma once
#include <map>
#include <vector>
#include <string>
#include "brain/robot_interface.hpp"
#include "interfaces/msg/marker_data.hpp"

class SoilRoutine {
public:
    explicit SoilRoutine(RobotInterface tools);

    bool run(double soil_threshold, const std::map<int, interfaces::msg::MarkerData>& markers);

private:
    RobotInterface tools_;
    
    // This declaration was likely missing or mismatched in your previous file
    bool callMovementService(const std::string &command, const std::vector<float> &positions);
};