#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <iostream>
#include <motion_ifc/DataConversion.h>

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
    void compute_interpolation_params(StateSpace pva0, StateSpace pvaf, double t0, double tf);
    void compute_interpolation_params(vector<double> x0, vector<double> dx0, vector<double> ddx0, vector<double> xf,
                                      vector<double> dxf, vector<double> ddxf, double t0, double tf);

    MatrixXd get_interpolated_x(double t);
    MatrixXd get_interpolated_dx(double t);
    MatrixXd get_interpolated_ddx(double t);
    StateSpace get_interpolated_pva(double t);

    inline double get_t0(){return t0+t_offset;}
    inline double get_tf(){return tf+t_offset;}

private:
    MatrixXd compute_t_mat(double t0 , double tf);

    MatrixXd Tmat;
    MatrixXd BoundaryConditions;
    MatrixXd Coefficients;

    double t0, tf;
    double t_offset;
};
#endif // TRAJECTORY_H
