#ifndef FORWARD_KINEMATICS_H_
#define FORWARD_KINEMATICS_H_

#include <armadillo>
#include <vector>

class ForwardKinematics {
public:
    virtual arma::mat computePosition() = 0;
};

#endif  // FORWARD_KINEMATICS_H_