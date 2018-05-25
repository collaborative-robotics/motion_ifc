#ifndef DATA_CONVERSION_H
#define DATA_CONVERSION_H

#include <eigen3/Eigen/Dense>
#include <tf_conversions/tf_kdl.h>
#include <motion_ifc/crtkCommon.h>

using namespace Eigen;

////
/// \brief The LinearizedPVA struct
///
struct StateSpace{
    VectorXd x, dx, ddx;
};


///////////
/// \brief The DataConversion class
///
class DataConversion{
public:
    DataConversion();
    template <typename T>
    StateSpace* serialize(T &data);
    template <typename T>
    void deserialize(T *data, StateSpace* pva);

    void resize(const uint &size){
        if(pva.x.rows() != size){
            pva.x.resize(size);
            pva.dx.resize(size);
            pva.ddx.resize(size);
        }
    }

    void set_x(VectorXd &x){
        resize(x.rows());
        pva.x = x;
    }

    void set_dx(VectorXd &dx){
        resize(dx.rows());
        pva.dx = dx;
    }

    void set_ddx(VectorXd &ddx){
        resize(ddx.rows());
        pva.ddx = ddx;
    }

    inline VectorXd get_x(){return pva.x;}
    inline VectorXd get_dx(){return pva.dx;}
    inline VectorXd get_ddx(){return pva.ddx;}
private:
    StateSpace pva;
};


#endif
