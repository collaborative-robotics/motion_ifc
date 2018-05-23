#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>

#define DEBUG 0

using namespace Eigen;
using namespace std;


/////////
/// \brief The Trajectory class
///
class Trajectory{
public:
    Trajectory();
    ~Trajectory();
    void compute_interpolation_params(VectorXd x0, VectorXd dx0, VectorXd ddx0, VectorXd xf, VectorXd dxf, VectorXd ddxf, double t0, double tf);
    void compute_interpolation_params(double x0, double dx0, double ddx0, double xf, double dxf, double ddxf, double t0, double tf);
    void compute_interpolation_params(vector<double> x0, vector<double> dx0, vector<double> ddx0, vector<double> xf,
                                      vector<double> dxf, vector<double> ddxf, double t0, double tf);

    MatrixXd get_interpolated_x(double t);
    MatrixXd get_interpolated_dx(double t);
    MatrixXd get_interpolated_ddx(double t);

    inline double get_t0(){return t0;}
    inline double get_tf(){return tf;}

private:
    MatrixXd compute_t_mat(double t0 , double tf);

    MatrixXd Tmat;
    MatrixXd BoundaryConditions;
    MatrixXd Coefficients;

    double t0, tf;
};

Trajectory::Trajectory() {
    Tmat.resize(6,6);
    BoundaryConditions.resize(6,1);
    Coefficients.resize(1,6);
}

Trajectory::~Trajectory() {
//    delete Tmat;
}

MatrixXd Trajectory::compute_t_mat(double t0, double tf){

    Tmat << 1, t0, pow(t0, 2),   pow(t0, 3),    pow(t0, 4),    pow(t0, 5),
            0,  1,       2*t0, 3*pow(t0, 2),  4*pow(t0, 3),  5*pow(t0, 4),
            0,  0,          2,         6*t0, 12*pow(t0, 2), 20*pow(t0, 3),
            1, tf, pow(tf, 2),   pow(tf, 3),    pow(tf, 4),    pow(tf, 5),
            0,  1,       2*tf, 3*pow(tf, 2),  4*pow(tf, 3),  5*pow(tf, 4),
            0,  0,          2,         6*tf, 12*pow(tf, 2), 20*pow(tf, 3);
    return Tmat;
}

void Trajectory::compute_interpolation_params(VectorXd x0,
                                                      VectorXd dx0,
                                                      VectorXd ddx0,
                                                      VectorXd xf,
                                                      VectorXd dxf,
                                                      VectorXd ddxf,
                                                      double t0,
                                                      double tf){
    this->t0 = t0;
    this->tf = tf;
    if (BoundaryConditions.cols() != x0.rows()){
        BoundaryConditions.resize(6, x0.rows());
        Coefficients.resize(x0.cols(), 6);
    }

    BoundaryConditions << x0.transpose(),
                         dx0.transpose(),
                        ddx0.transpose(),
                          xf.transpose(),
                         dxf.transpose(),
                        ddxf.transpose();

    compute_t_mat(t0, tf);
    Coefficients = Tmat.inverse() * BoundaryConditions;
    if (DEBUG){
        std::cout << "Size of x0 \n" << x0.rows() << x0.cols() << std::endl;
        std::cout << "B \n" << BoundaryConditions << std::endl;
        std::cout << "Size of B \n" << BoundaryConditions.rows() << BoundaryConditions.cols() << std::endl;
        std::cout << "C \n" << Coefficients << std::endl;
    }
}

MatrixXd Trajectory::get_interpolated_x(double t){
    VectorXd X(BoundaryConditions.cols());
    MatrixXd T(6,1);
    T << 1, t, pow(t, 2),   pow(t, 3),    pow(t, 4),    pow(t, 5);
    if (DEBUG){
        std::cout << "Size of T \n" << T.rows() << T.cols() << std::endl;
        std::cout << "Size of C \n" << Coefficients.rows() << Coefficients.cols() << std::endl;
    }
    X = Coefficients.transpose() * T;
    return X;
}

MatrixXd Trajectory::get_interpolated_dx(double t){
    VectorXd X(BoundaryConditions.cols());
    MatrixXd T(6,1);
    T << 0, 1, 2*t,   3*pow(t, 2),    4*pow(t, 3),    5*pow(t, 4);
    if (DEBUG){
        std::cout << "Size of T \n" << T.rows() << T.cols() << std::endl;
        std::cout << "Size of C \n" << Coefficients.rows() << Coefficients.cols() << std::endl;
    }
    X = Coefficients.transpose() * T;
    return X;
}

MatrixXd Trajectory::get_interpolated_ddx(double t){
    VectorXd X(BoundaryConditions.cols());
    MatrixXd T(6,1);
    T << 0, 1, 2,   6*t,    12*pow(t, 2),    20*pow(t, 3);
    if (DEBUG){
        std::cout << "Size of T \n" << T.rows() << T.cols() << std::endl;
        std::cout << "Size of C \n" << Coefficients.rows() << Coefficients.cols() << std::endl;
    }
    X = Coefficients.transpose() * T;
    return X;
}
#endif // INTERPOLATE_H
