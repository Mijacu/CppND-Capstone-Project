#include "Coordinates.h"

Coordinates::Coordinates(double x) {
    coordinates_.set_size(1);
    coordinates_ << x;
}

Coordinates::Coordinates(double x, double y) {
    coordinates_.set_size(2,1);
    coordinates_ << x << arma::endr
                 << y;
}

Coordinates::Coordinates(double x, double y, double z) {
    coordinates_.set_size(3,1);
    coordinates_  << x << arma::endr
                  << y << arma::endr
                  << z;
}

void Coordinates::print() {
    coordinates_.print();
}

void Coordinates::print(std::string header) {
    coordinates_.print(header);
}