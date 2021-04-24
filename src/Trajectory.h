#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include <vector>

class Trajectory {
public:
    virtual std::vector<std::vector<double>> generateTrajectory() = 0;

protected:
    std::vector<std::vector<double>> trajectory_;
    int total_positions_ = 200;
};

#endif  /* TRAJECTORY_H_ */