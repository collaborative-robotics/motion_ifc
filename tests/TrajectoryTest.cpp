#include "motion_ifc/Trajectory.h"
#include <iostream>

using namespace std;
int main() {
    std::cout << "Testing Interpolation Logic using Eigen \n" << std::endl;
    Trajectory obj;
    RowVectorXd x0, dx0, ddx0, xf, dxf, ddxf;
      x0.resize(3);
     dx0.resize(3);
    ddx0.resize(3);
      xf.resize(3);
     dxf.resize(3);
    ddxf.resize(3);

      x0 << 0.0, 0.0, 0.0;
     dx0 << 0.0, 0.0, 0.0;
    ddx0 << 0.0, 0.0, 0.0;
      xf << 1.0, 5.0, -.5;
     dxf << 0.0, 0.0, 0.0;
    ddxf << 0.0, 0.0, 0.0;

    double t0, tf;
    t0 = 0.0;
    tf = 1.0;
    obj.compute_interpolation_params(x0 ,dx0, ddx0, xf, dxf, ddxf, t0, tf);
    MatrixXd p, v, a;
    double t = 0.0;
    while(t < 2 * tf){
        p = obj.get_interpolated_x(t);
        v = obj.get_interpolated_dx(t);
        a = obj.get_interpolated_ddx(t);
//        std::cout << "x: \n" << p << "\nv: \n" << v << "\na: \n" << a << "\n at t: " << t << std::endl;
        std::cout << "x: \n" << p << "\n at t: " << t << std::endl;
        std::cout << "--------------------------------------------\n";
        t = t + 0.01;
        usleep(100000);
    }
    return 0;
}
