#include <iostream>
#include <vector>

#include "Position.h"
#include "RoboticArm.h"
#include "CircularTrajectory.h"

int main() {
    std::vector<double> links {10.0, 5.0};
    RoboticArm robotic_arm(links);

    Position position(1.0, 2.0, 4.0);
    position.print("position: ");

    arma::mat postion = robotic_arm.computePosition();
    position.print("Computed positions: ");

    std::vector<double> desired_position{0.0, 15.0};
    robotic_arm.computeAngles(desired_position);
    robotic_arm.printAngles();

    desired_position[0] = -15.0;
    desired_position[1] = 0.0;
    robotic_arm.computeAngles(desired_position);
    robotic_arm.printAngles();

    CircularTrajectory circular_trajectory;
    std::vector<std::vector<double>> trajectory = circular_trajectory.generateTrajectory();
    for (std::vector<double> position : trajectory) {
        robotic_arm.computeAngles(position);
        robotic_arm.printAngles();
    }
    
    return 0;
}