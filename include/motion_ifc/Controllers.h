#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include <motion_ifc/crtkCommon.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <motion_ifc/DataConversion.h>
#include <motion_ifc/Trajectory.h>
#include <motion_ifc/FcnHandle.h>

using namespace Eigen;

class Interpolate: public DataConversion{
public:
    Interpolate();
    ~Interpolate(){
//        delete temp;
        delete method_map["interpolate_cp"];
    }
    void interpolate_cp(_cp_data_type &data);
    void interpolate_cr(_cr_data_type &data);
    void interpolate_cv(_cv_data_type &data);
    void interpolate_cf(_cf_data_type &data);

    void interpolate_jp(_jp_data_type &data);
    void interpolate_jr(_jr_data_type &data);
    void interpolate_jv(_jv_data_type &data);
    void interpolate_jf(_jf_data_type &data);

    std::map<std::string, FcnHandleBase*> method_map;
//    FcnHandle<_cp_data_type>* temp;

};

Interpolate::Interpolate(){
//    temp = new FcnHandle<_cp_data_type>(&Interpolate::interpolate_cp, this);
//    temp->assign_fcn(&Interpolate::interpolate_cp, this);
//    method_map["interpolate_cp"] = temp;
    method_map["interpolate_cp"] = new FcnHandle<_cp_data_type>(&Interpolate::interpolate_cp, this);
}

void Interpolate::interpolate_cp(_cp_data_type &data){
    std::cout << "Passed Transform is: \n" << data.transform.translation << std::endl;
}

class Move{
public:
    Move();
    void move_cp(_cp_data_type &data);
    void move_cr(_cr_data_type &data);

    void move_jp(_jp_data_type &data);
    void move_jr(_jr_data_type &data);
};

class Servo{
public:
    Servo();
    void servo_cp(_cp_data_type &data);
    void servo_cr(_cr_data_type &data);
    void servo_cv(_cv_data_type &data);
    void servo_cf(_cf_data_type &data);

    void servo_jp(_jp_data_type &data);
    void servo_jr(_jr_data_type &data);
    void servo_jv(_jv_data_type &data);
    void servo_jf(_jf_data_type &data);
};

class Controllers: public Interpolate, Move, Servo{
public:
    Controllers();

private:
};


#endif // CONTROLLERS_H
