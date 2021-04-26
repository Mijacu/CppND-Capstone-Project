#ifndef CIRCULAR_TRAJECTORY_H_
#define CIRCULAR_TRAJECTORY_H_

#include <iostream>
#include <math.h>
#include <vector>

#include "Trajectory.h"

#ifndef PI
#define PI 3.1415926535
#endif

class CircularTrajectory : public Trajectory {
public:
    CircularTrajectory() {}; 
    ~CircularTrajectory() {};      
    std::vector<std::vector<double>> generateTrajectory() override;
};

#endif