#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include <motion_ifc/crtkCommon.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <motion_ifc/DataConversion.h>
#include <motion_ifc/Trajectory.h>
#include <motion_ifc/FcnHandle.h>

using namespace Eigen;

typedef std::map<std::string, boost::shared_ptr<FcnHandleBase> > _method_map_type;

class Interpolate: public DataConversion{
public:
    Interpolate();
    ~Interpolate(){
    }
    void interpolate_cp(_cp_data_type &data);
    void interpolate_cr(_cr_data_type &data);
    void interpolate_cv(_cv_data_type &data);
    void interpolate_cf(_cf_data_type &data);

    void interpolate_jp(_jp_data_type &data);
    void interpolate_jr(_jr_data_type &data);
    void interpolate_jv(_jv_data_type &data);
    void interpolate_jf(_jf_data_type &data);

    boost::shared_ptr<FcnHandleBase> get_method_by_name(std::string method_name){
        return method_map[method_name];
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }


private:
    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};

Interpolate::Interpolate(){
    method_map["interpolate_cp"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cp_data_type>(&Interpolate::interpolate_cp, this));
    method_map["interpolate_cr"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cr_data_type>(&Interpolate::interpolate_cr, this));
    method_map["interpolate_cv"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cv_data_type>(&Interpolate::interpolate_cv, this));
    method_map["interpolate_cf"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cf_data_type>(&Interpolate::interpolate_cf, this));

    method_map["interpolate_jp"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_jp_data_type>(&Interpolate::interpolate_jp, this));
    method_map["interpolate_jr"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_jr_data_type>(&Interpolate::interpolate_jr, this));
    method_map["interpolate_jv"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_jv_data_type>(&Interpolate::interpolate_jv, this));
    method_map["interpolate_jf"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_jf_data_type>(&Interpolate::interpolate_jf, this));
}

void Interpolate::interpolate_cp(_cp_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
    serialize(data);
    cout << "Passed Data is \n" << x << std::endl;
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
    serialize(data);
    cout << "Passed Data is \n" << x << std::endl;
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

    boost::shared_ptr<FcnHandleBase> get_method_by_name(std::string method_name){
        return method_map[method_name];
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }


private:
    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};

Move::Move(){
    method_map["move_cp"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_cp_data_type>(&Move::move_cp, this));
    method_map["move_cr"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_cr_data_type>(&Move::move_cr, this));
    method_map["move_jp"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_jp_data_type>(&Move::move_jp, this));
    method_map["move_jr"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_jr_data_type>(&Move::move_jr, this));
}

void Move::move_cp(_cp_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Move::move_cr(_cr_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Move::move_jp(_jp_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Move::move_jr(_jr_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}


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

    boost::shared_ptr<FcnHandleBase> get_method_by_name(std::string method_name){
        return method_map[method_name];
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }


private:
    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};

Servo::Servo(){
    method_map["servo_cp"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_cp_data_type>(&Servo::servo_cp, this));
    method_map["servo_cr"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_cr_data_type>(&Servo::servo_cr, this));
    method_map["servo_cv"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_cv_data_type>(&Servo::servo_cv, this));
    method_map["servo_cf"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_cf_data_type>(&Servo::servo_cf, this));
    method_map["servo_jp"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_jp_data_type>(&Servo::servo_jp, this));
    method_map["servo_jr"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_jr_data_type>(&Servo::servo_jr, this));
    method_map["servo_jv"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_jv_data_type>(&Servo::servo_jv, this));
    method_map["servo_jf"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_jf_data_type>(&Servo::servo_jf, this));
}

void Servo::servo_cp(_cp_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Servo::servo_cr(_cr_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Servo::servo_cv(_cv_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Servo::servo_cf(_cf_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Servo::servo_jp(_jp_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Servo::servo_jr(_jr_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Servo::servo_jv(_jv_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}

void Servo::servo_jf(_jf_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
}


class Controllers: public Interpolate, Move, Servo{
public:
    Controllers();

    boost::shared_ptr<FcnHandleBase> get_method_by_name(std::string method_name){
        std::vector<std::string> x = split_str(method_name, '_');
        std::string mode = x[0];
        char op_space = x[1][0];
        char controller = x[1][1];

        if (mode.compare("interpolate") == 0){
            return Interpolate::get_method_by_name(method_name);
        }
        if (mode.compare("move") == 0){
            return Move::get_method_by_name(method_name);
        }
        if (mode.compare("servo") == 0){
            return Servo::get_method_by_name(method_name);
        }
    }

private:
};

Controllers::Controllers(){
}


#endif // CONTROLLERS_H
