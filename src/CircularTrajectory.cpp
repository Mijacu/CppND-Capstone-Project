#include "CircularTrajectory.h"

std::vector<std::vector<double>> CircularTrajectory::generateTrajectory() {
    double x, y;
    double magnitude = 12.5;
    trajectory_.clear();
    for(double theta = 0; theta < PI; theta += PI / total_positions_) {
        std::cout << theta <<std::endl;
        x = magnitude * cos(theta);
        y = magnitude * sin(theta);
        std::cout << "x= " << x << " y= " << y << std::endl;
        trajectory_.push_back(std::vector<double> {x, y});
    }
    return trajectory_;
}