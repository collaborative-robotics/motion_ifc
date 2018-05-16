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
        for (_method_iterator = method_map.begin(); _method_iterator != method_map.end() ; _method_iterator++){
            delete _method_iterator->second;
        }
    }
    void interpolate_cp(_cp_data_type &data);
    void interpolate_cr(_cr_data_type &data);
    void interpolate_cv(_cv_data_type &data);
    void interpolate_cf(_cf_data_type &data);

    void interpolate_jp(_jp_data_type &data);
    void interpolate_jr(_jr_data_type &data);
    void interpolate_jv(_jv_data_type &data);
    void interpolate_jf(_jf_data_type &data);

    template<typename D>
    FcnHandle<D>* get_method_by_name(std::string method_name){
        return (FcnHandle<D> *)method_map[method_name];
    }

    template<typename D>
    void get_method_by_name(std::string method_name, FcnHandleBase** fcn){
        *fcn = (FcnHandle<D> *)method_map[method_name];
    }


private:
    std::map<std::string, FcnHandleBase*> method_map;
    std::map<std::string, FcnHandleBase*>::iterator _method_iterator;
};

Interpolate::Interpolate(){
    method_map["interpolate_cp"] = new FcnHandle<_cp_data_type>(&Interpolate::interpolate_cp, this);
    method_map["interpolate_cr"] = new FcnHandle<_cr_data_type>(&Interpolate::interpolate_cr, this);
    method_map["interpolate_cv"] = new FcnHandle<_cv_data_type>(&Interpolate::interpolate_cv, this);
    method_map["interpolate_cf"] = new FcnHandle<_cf_data_type>(&Interpolate::interpolate_cf, this);

    method_map["interpolate_jp"] = new FcnHandle<_jp_data_type>(&Interpolate::interpolate_jp, this);
    method_map["interpolate_jr"] = new FcnHandle<_jr_data_type>(&Interpolate::interpolate_jr, this);
    method_map["interpolate_jv"] = new FcnHandle<_jv_data_type>(&Interpolate::interpolate_jv, this);
    method_map["interpolate_jf"] = new FcnHandle<_jf_data_type>(&Interpolate::interpolate_jf, this);
}

void Interpolate::interpolate_cp(_cp_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
    std::cout << "Passed Transform is: \n" << data.transform.translation << std::endl;
}

void Interpolate::interpolate_cr(_cr_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Interpolate::interpolate_cv(_cv_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Interpolate::interpolate_cf(_cf_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Interpolate::interpolate_jp(_jp_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Interpolate::interpolate_jr(_jr_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Interpolate::interpolate_jv(_jv_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Interpolate::interpolate_jf(_jf_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
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
