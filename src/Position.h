#ifndef Position_H_
#define Position_H_

#include <armadillo>

class Position {
public: 
    // constructors / destructors
    Position(arma::mat position);
    Position(double x);
    Position(double x, double y);
    Position(double x, double y, double z);
    arma::mat getPositionMatrix();
    void setPositionMatrix(arma::mat position);
    void print();
    void print(std::string header);
    ~Position() = default;

private:
    arma::mat position_;
};
#endif /* Position_H_ */
