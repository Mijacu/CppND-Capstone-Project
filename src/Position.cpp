#include "Position.h"

Position::Position(arma::mat position) {
    position_ = position; 
}

Position::Position(double x) {
    position_.set_size(1);
    position_ << x;
}

Position::Position(double x, double y) {
    position_.set_size(2,1);
    position_ << x << arma::endr
                 << y;
}

Position::Position(double x, double y, double z) {
    position_.set_size(3,1);
    position_  << x << arma::endr
                  << y << arma::endr
                  << z;
}

arma::mat Position::getPositionMatrix() {
    return position_;
}
    
void Position::setPositionMatrix(arma::mat position) {
    position_ = position;
}

void Position::print() {
    position_.print();
}

void Position::print(std::string header) {
    position_.print(header);
}