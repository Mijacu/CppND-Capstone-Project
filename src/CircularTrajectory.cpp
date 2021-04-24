#include "CircularTrajectory.h"

std::vector<std::vector<double>> CircularTrajectory::generateTrajectory() {
    double x, y, magnitude;
    trajectory_.clear();
    for(int theta = 0; theta < PI; theta += PI / total_positions_) {
        x = magnitude * cos(theta);
        y = magnitude * sin(theta);
        trajectory_.push_back(std::vector<double> {x, y})
    }
    return trajectory_;
}