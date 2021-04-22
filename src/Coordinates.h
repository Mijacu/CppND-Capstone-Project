#ifndef COORDINATES_H_
#define COORDINATES_H_

#include <armadillo>

class Coordinates {
public: 
    // constructors / destructors
    Coordinates(double x);
    Coordinates(double x, double y);
    Coordinates(double x, double y, double z);
    void print();
    void print(std::string header);
    ~Coordinates() = default;

private:
    arma::mat coordinates_;
};
#endif /* COORDINATES_H_ */
