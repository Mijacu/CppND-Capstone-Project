#include "Conversions.h"

double Conversions::gradesToRadians(double angle_in_grades) {
    return angle_in_grades * PI / 180.0;
}

double Conversions::radiansToGrades(double angle_in_radians) {
    return angle_in_radians * 180.0 / PI;
}