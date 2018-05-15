#ifndef INTERPOLATE_H
#define INTERPOLATE_H

#include <eigen3/Eigen/Dense>
#include <vector>

using namespace Eigen;
using namespace std;

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

private:
    MatrixXd compute_t_mat(double t0 , double tf);

    MatrixXd Tmat;
    MatrixXd BoundaryConditions;
    MatrixXd Coefficients;
};
#endif // INTERPOLATE_H
