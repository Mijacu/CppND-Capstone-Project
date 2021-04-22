#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#ifndef PI
#define PI 3.1415926535
#endif

class Conversions {
public:
    Conversions() = default;
    double gradesToRadians(double angle_in_grades);
    double radiansToGrades(double angle_in_radians);
};

#endif /* CONVERSIONS_H_ */