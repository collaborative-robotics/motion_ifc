#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include <motion_ifc/crtkCommon.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <motion_ifc/DataConversion.h>
#include <motion_ifc/Trajectory.h>
#include <motion_ifc/FcnHandle.h>

using namespace Eigen;

typedef std::map<std::string, FcnHandleBase*> _method_map_type;

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

    void get_method_by_name_auto_specialized(std::string method_name, FcnHandleBase** fcn){
        std::vector<std::string> x = split_str(method_name, '_');
        char op_space = x[1][0];
        if (op_space == 'c'){
            std::cout << "Cartesian Space Specified" << std::endl;
            *fcn = (FcnHandle<_cp_data_type> *)method_map[method_name];
        }
        else if (op_space == 'j'){
            std::cout << "Joint Space Specified" << std::endl;
            *fcn = (FcnHandle<_jp_data_type> *)method_map[method_name];
        }
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }


private:
    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
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

    template<typename D>
    FcnHandle<D>* get_method_by_name(std::string method_name){
        return (FcnHandle<D> *)method_map[method_name];
    }

    template<typename D>
    void get_method_by_name(std::string method_name, FcnHandleBase** fcn){
        *fcn = (FcnHandle<D> *)method_map[method_name];
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }


private:
    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};

Move::Move(){
    method_map["move_cp"] = new FcnHandle<_cp_data_type>(&Move::move_cp, this);
    method_map["move_cr"] = new FcnHandle<_cr_data_type>(&Move::move_cr, this);
    method_map["move_jp"] = new FcnHandle<_jp_data_type>(&Move::move_jp, this);
    method_map["move_jr"] = new FcnHandle<_jr_data_type>(&Move::move_jr, this);
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

    template<typename D>
    FcnHandle<D>* get_method_by_name(std::string method_name){
        return (FcnHandle<D> *)method_map[method_name];
    }

    template<typename D>
    void get_method_by_name(std::string method_name, FcnHandleBase** fcn){
        *fcn = (FcnHandle<D> *)method_map[method_name];
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }


private:
    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};

Servo::Servo(){
    method_map["servo_cp"] = new FcnHandle<_cp_data_type>(&Servo::servo_cp, this);
    method_map["servo_cr"] = new FcnHandle<_cr_data_type>(&Servo::servo_cr, this);
    method_map["servo_cv"] = new FcnHandle<_cv_data_type>(&Servo::servo_cv, this);
    method_map["servo_cf"] = new FcnHandle<_cf_data_type>(&Servo::servo_cf, this);

    method_map["servo_jp"] = new FcnHandle<_jp_data_type>(&Servo::servo_jp, this);
    method_map["servo_jr"] = new FcnHandle<_jr_data_type>(&Servo::servo_jr, this);
    method_map["servo_jv"] = new FcnHandle<_jv_data_type>(&Servo::servo_jv, this);
    method_map["servo_jf"] = new FcnHandle<_jf_data_type>(&Servo::servo_jf, this);
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

    template<typename D>
    FcnHandle<D>* get_method_by_name(std::string method_name){
        return (FcnHandle<D> *)method_map[method_name];
    }

    template<typename D>
    void get_method_by_name(std::string method_name, FcnHandleBase** fcn){
        *fcn = (FcnHandle<D> *)method_map[method_name];
    }

private:
    _method_map_type method_map;
};

Controllers::Controllers(){
    std::cout << "Size " << method_map.size() << std::endl;
    _method_map_type* temp_map;
    temp_map = Interpolate::get_method_names_map();
    method_map.insert(temp_map->begin(), temp_map->end());
    temp_map = Move::get_method_names_map();
    method_map.insert(temp_map->begin(), temp_map->end());
    temp_map = Servo::get_method_names_map();
    method_map.insert(temp_map->begin(), temp_map->end());
}


#endif // CONTROLLERS_H
