#include <iostream>
#include <vector>

#include "Position.h"
#include "RoboticArm.h"
#include "CircularTrajectory.h"

int main() {
    // Initialiaze a 2 DOF robotic arm
    std::vector<double> links {10.0, 5.0};
    RoboticArm robotic_arm(links);

    // Compute initial position
    arma::mat position = robotic_arm.computePosition();
    position.print("Computed positions: ");

    // Test inverse kinematics on two desired positions
    std::vector<double> desired_position{0.0, 15.0};
    robotic_arm.computeAngles(desired_position);
    robotic_arm.printAngles();

    desired_position[0] = -15.0;
    desired_position[1] = 0.0;
    robotic_arm.computeAngles(desired_position);
    robotic_arm.printAngles();

    // Generate a circular trajectory and calculate the angles to carry out that trajectory
    CircularTrajectory circular_trajectory;
    std::vector<std::vector<double>> trajectory = circular_trajectory.generateTrajectory();
    for (std::vector<double> position : trajectory) {
        robotic_arm.computeAngles(position);
        std::cout << "x= " << position[0] << " y= " << position[1] << std::endl;
        robotic_arm.printAngles();
    }

    return 0;
}