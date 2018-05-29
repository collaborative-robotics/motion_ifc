#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <motion_ifc/ControllerData.h>


using namespace Eigen;

class ControllerManager: public RobotCmd, public RobotState{
    friend class Interpolate;
    friend class Move;
    friend class Servo;

public:
    ControllerManager();
    static inline void clear_initialized_flag(){_initialized = false;}

private:
    CtrlrBasePtr cpCtrl;
    CtrlrBasePtr crCtrl;
    CtrlrBasePtr cvCtrl;
    CtrlrBasePtr cfCtrl;
    CtrlrBasePtr jpCtrl;
    CtrlrBasePtr jrCtrl;
    CtrlrBasePtr jvCtrl;
    CtrlrBasePtr jfCtrl;

    ros::Rate rate;
    void execute();
    boost::thread execTh;

    vector<CtrlrBasePtr> vec_ctrlrs;
    static bool _initialized;
    void bind_robot_io(string interface_name, CtrlrBasePtr &ctrlrBase);
};

///
/// \brief The Interpolate class
///
class Interpolate: public ControllerManager{
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

    FcnHandleBasePtr get_method_by_name(std::string method_name){
        return method_map[method_name];
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }

private:
    void bind_robot_io(string interface_name, CtrlrBasePtr &ctrlrBase);

    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};


///
/// \brief The Move class
///
class Move: public ControllerManager{
public:
    Move();
    void move_cp(_cp_data_type &data);
    void move_cr(_cr_data_type &data);

    void move_jp(_jp_data_type &data);
    void move_jr(_jr_data_type &data);

    FcnHandleBasePtr get_method_by_name(std::string method_name){
        return method_map[method_name];
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }


private:
    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};


///
/// \brief The Servo class
///
class Servo: public ControllerManager{
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

    FcnHandleBasePtr get_method_by_name(std::string method_name){
        return method_map[method_name];
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }


private:
    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};


///
/// \brief The Controllers class
///
class Controllers:
        public Interpolate,
        public Move,
        public Servo{
public:
    Controllers();
    FcnHandleBasePtr get_method_by_name(std::string method_name);
};

#endif // CONTROLLERS_H
