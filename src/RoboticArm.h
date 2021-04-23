#ifndef ROBOTIC_ARM_H_
#define ROBOTIC_ARM_H_

#include <armadillo>
#include <math.h>

#include "Conversions.h"
#include "ForwardKinematics.h"

class RoboticArm : public ForwardKinematics{
private:
    Conversions conversions;
};

#endif /* ROBOTIC_ARM_H_ */