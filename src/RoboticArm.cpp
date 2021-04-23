#include "RoboticArm.h"

arma::mat RoboticArm::computePosition() {
    double x = cos(angles_[0]) * links_[0] + cos(angles_[0] + angles_[1]) * links_[1];
    double y = sin(angles_[0]) * links_[0] + sin(angles_[0] + angles_[1]) * links_[1];
    arma::mat position(2,1, arma::fill::zeros);
    position << x << arma::endr
             << y ;   
    return position;
}

void RoboticArm::computeAngles(std::vector<double>& position) {
    angles_[1] = acos((pow(position[0], 2) + pow(position[1], 2) - \
            pow(links_[0], 2) - pow(links_[1], 2)) / (2.0 * links_[0] * links_[1]));
    angles_[0] = atan2(position[1], position[0]) - atan2((links_[1] * sin(angles_[1])), \
            (links_[0] + links_[1] * cos(angles_[1])));
}

void RoboticArm::printAngles() {
    std::cout << "Angles: " <<std::endl;
    for(double angle : angles_) {
        std::cout << "\t" << conversions_.radiansToGrades(angle) << std::endl;
    }
}