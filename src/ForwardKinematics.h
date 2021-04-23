#ifndef FORWARD_KINEMATICS_H_
#define FORWARD_KINEMATICS_H_

class ForwardKinematics {
public:
    virtual arma::mat computePosition(const vector& angles_in_grades) = 0;
}

#endif  // FORWARD_KINEMATICS_H_