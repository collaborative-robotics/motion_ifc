#ifndef CONTROLLERS_H
#define CONTROLLERS_H
#include <motion_ifc/crtkCommon.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <motion_ifc/DataConversion.h>
#include <motion_ifc/Trajectory.h>
#include <motion_ifc/FcnHandle.h>
#include <motion_ifc/RobotCmdIfc.h>
#include <motion_ifc/RobotStateIfc.h>

using namespace Eigen;

typedef std::map<std::string, boost::shared_ptr<FcnHandleBase> > _method_map_type;

struct ControllerDataBase: public DataConversion{
public:
    ControllerDataBase(){}
    boost::shared_ptr<FcnHandleBase> robot_cmd_method;
    boost::shared_ptr<FcnHandleBase> robot_state_method;
    Trajectory interpolater;

    bool active;

    inline void set_active(){active = true;}
    inline void set_idle(){active = false;}
    inline bool is_active(){return active;}

    virtual void cmd_robot(double t){}
};

template <typename D>
struct ControllerData: public ControllerDataBase{
    ControllerData(std::string interface_name, RobotCmdIfc &rCmdIfc, RobotStateIfc &rStateIfc);

    D data;
    virtual void cmd_robot(double t){
        interpolater.get_interpolated_x(t);
        deserialize(&data);
        (*robot_cmd_method)(data);
    }
};

template <typename D>
ControllerData<D>::ControllerData(std::string interface_name, RobotCmdIfc &rCmdIfc, RobotStateIfc &rStateIfc){
    active = false;
    std::vector<std::string> x = split_str(interface_name, '/');
    std::vector<std::string> crtk_str = split_str(x.back(), '_');
    char op_space = crtk_str[1][0];
    char controller = crtk_str[1][1];

    char cmd_controller = controller;
    char state_controller = controller;

    if (op_space == 'c'){
        switch (controller){
        case 'r':
            cmd_controller = 'p';
            state_controller = 'p';
            break;
        }
    }
    else if (op_space == 'j'){
        state_controller = 's';
        switch (controller){
        case 'r':
            cmd_controller = 'p';
            break;
        }
    }
    std::string robot_state_search_str = "measured_" + op_space + state_controller;
    std::string robot_cmd_search_str = "servo_" + op_space + cmd_controller;

    robot_cmd_method = rCmdIfc.get_method_by_name(robot_cmd_search_str);
    robot_state_method = rStateIfc.get_method_by_name(robot_state_search_str);
}

typedef boost::shared_ptr<ControllerDataBase> CtrlrBasePtr;

/////////////////////////////////////////////////////////////////

class Interpolate{
public:
    Interpolate(RobotCmdIfc &rCmdIfc,RobotStateIfc &rStateIfc);
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
    CtrlrBasePtr cpCtrl;
    CtrlrBasePtr crCtrl;
    CtrlrBasePtr cvCtrl;
    CtrlrBasePtr cfCtrl;
    CtrlrBasePtr jpCtrl;
    CtrlrBasePtr jrCtrl;
    CtrlrBasePtr jvCtrl;
    CtrlrBasePtr jfCtrl;

    void execute();
    vector<CtrlrBasePtr> vCtrlrs;

    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};

Interpolate::Interpolate(RobotCmdIfc &rCmdIfc,RobotStateIfc &rStateIfc){
    method_map["interpolate_cp"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cp_data_type>(&Interpolate::interpolate_cp, this));
    method_map["interpolate_cr"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cr_data_type>(&Interpolate::interpolate_cr, this));
    method_map["interpolate_cv"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cv_data_type>(&Interpolate::interpolate_cv, this));
    method_map["interpolate_cf"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cf_data_type>(&Interpolate::interpolate_cf, this));

    method_map["interpolate_jp"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_jp_data_type>(&Interpolate::interpolate_jp, this));
    method_map["interpolate_jr"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_jr_data_type>(&Interpolate::interpolate_jr, this));
    method_map["interpolate_jv"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_jv_data_type>(&Interpolate::interpolate_jv, this));
    method_map["interpolate_jf"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_jf_data_type>(&Interpolate::interpolate_jf, this));

    cpCtrl = CtrlrBasePtr(new ControllerData<_cp_data_type>("interpolate_cp", rCmdIfc, rStateIfc));
    crCtrl = CtrlrBasePtr(new ControllerData<_cr_data_type>("interpolate_cr", rCmdIfc, rStateIfc));
    cvCtrl = CtrlrBasePtr(new ControllerData<_cv_data_type>("interpolate_cv", rCmdIfc, rStateIfc));
    cfCtrl = CtrlrBasePtr(new ControllerData<_cf_data_type>("interpolate_cf", rCmdIfc, rStateIfc));
    jpCtrl = CtrlrBasePtr(new ControllerData<_jp_data_type>("interpolate_jp", rCmdIfc, rStateIfc));
    jrCtrl = CtrlrBasePtr(new ControllerData<_jr_data_type>("interpolate_jr", rCmdIfc, rStateIfc));
    jvCtrl = CtrlrBasePtr(new ControllerData<_jv_data_type>("interpolate_jv", rCmdIfc, rStateIfc));
    jfCtrl = CtrlrBasePtr(new ControllerData<_jf_data_type>("interpolate_jf", rCmdIfc, rStateIfc));
}

void Interpolate::interpolate_cp(_cp_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
    cpCtrl->serialize(data);
    double t0 = time(NULL);
    double tf = t0 + 1.0;
    cpCtrl->interpolater.compute_interpolation_params(cpCtrl->x,
                                                      cpCtrl->x,
                                                      cpCtrl->x,
                                                      cpCtrl->x,
                                                      cpCtrl->x,
                                                      cpCtrl->x,
                                                      t0,
                                                      tf);
    cout << "Passed Data is \n" << cpCtrl->x << std::endl;
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
    jpCtrl->serialize(data);
    cout << "Passed Data is \n" << jpCtrl->x << std::endl;
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

void Interpolate::execute(){
    while (ros::ok()){
        for (std::vector<CtrlrBasePtr>::iterator it = vCtrlrs.begin() ; it != vCtrlrs.end() ; it++){
            if ((*it)->is_active()){
                double t = time(NULL);
                while ( t >= (*it)->interpolater.get_t0() && t<= (*it)->interpolater.get_tf()){
                    t = time(NULL);
                    (*it)->cmd_robot(t);
                }
                (*it)->set_idle();
            }
        }
    }
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
    Controllers(RobotCmdIfc &rCmdIfc, RobotStateIfc &rStateIfc);

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

Controllers::Controllers(RobotCmdIfc &rCmdIfc, RobotStateIfc &rStateIfc): Interpolate(rCmdIfc, rStateIfc){
}


#endif // CONTROLLERS_H
