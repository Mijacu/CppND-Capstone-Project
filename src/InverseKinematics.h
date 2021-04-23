#ifndef INVERSE_KINEMATICS_H_
#define INVERSE_KINEMATICS_H_

#include <armadillo>
#include <vector>

class InverseKinematics {
public:
    virtual void computeAngles(std::vector<double>& position) = 0;
};

#endif  //INVERSE_KINEMATICS_H_