#ifndef ROBOTIC_ARM_H_
#define ROBOTIC_ARM_H_

#include <armadillo>
#include <math.h>
#include <vector>

#include "Conversions.h"
#include "InverseKinematics.h"
#include "ForwardKinematics.h"

class RoboticArm : public ForwardKinematics, public InverseKinematics {
public:
    RoboticArm() = delete;
    ~RoboticArm() = default;
    RoboticArm(std::vector<double>& links) : links_(links), angles_{0.0, 0.0} {};
    RoboticArm(const RoboticArm&) = delete;
    RoboticArm& operator=(const RoboticArm&) = delete;
    void computeAngles(std::vector<double>& position) override;
    arma::mat computePosition() override;
    void printAngles();

private:
    Conversions conversions_;
    const std::vector<double> links_;
    std::vector<double> angles_;  // Angles in radians
};

#endif /* ROBOTIC_ARM_H_ */