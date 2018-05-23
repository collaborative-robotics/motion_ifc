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
#include <boost/thread.hpp>

using namespace Eigen;
////////
/// \brief The ControllerDataBase struct
///
struct ControllerDataBase: public DataConversion{
public:
    ControllerDataBase(){}
    FcnHandleBasePtr robot_cmd_method;
    FcnHandleBasePtr robot_state_method;
    Trajectory interpolater;

    bool active;

    inline void set_active(){active = true;}
    inline void set_idle(){active = false;}
    inline bool is_active(){return active;}

    virtual void cmd_robot(double t){}
};

/////
///
///
template <typename D, typename S>
struct ControllerData: public ControllerDataBase{
    ControllerData(std::string interface_name);

    D cmd_data;
    S state_data;
    virtual void cmd_robot(double t){
        interpolater.get_interpolated_x(t);
        deserialize(&state_data);
        (*robot_cmd_method)(state_data);
    }
};

template <typename D, typename S>
ControllerData<D, S>::ControllerData(std::string interface_name){
    active = false;
}

///////
/// \brief CtrlrBasePtr
///
typedef boost::shared_ptr<ControllerDataBase> CtrlrBasePtr;

class ControllerDataIfc{
public:
    ControllerDataIfc(){}
    CtrlrBasePtr create_controller_data_ifc(string interface_name, RobotCmdIfcConstPtr rCmdIfcPtr, RobotStateIfcConstPtr rStateIfc);
};

CtrlrBasePtr ControllerDataIfc::create_controller_data_ifc(string interface_name, RobotCmdIfcConstPtr rCmdIfc, RobotStateIfcConstPtr rStateIfc){
    std::vector<std::string> x = split_str(interface_name, '/');
    std::vector<std::string> crtk_str = split_str(x.back(), '_');
    char op_space = crtk_str[1][0];
    char controller = crtk_str[1][1];

    char cmd_controller = controller;
    char state_controller = controller;

    CtrlrBasePtr ctrlrBase;
    if (op_space == 'c'){
        switch (controller){
        case 'p':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_cp_data_type,  _cp_data_type>(interface_name));
            break;
        case 'r':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_cr_data_type,  _cr_data_type>(interface_name));
            cmd_controller = 'p';
            state_controller = 'p';
            break;
        case 'v':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_cv_data_type,  _cv_data_type>(interface_name));
            break;
        case 'f':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_cf_data_type,  _cf_data_type>(interface_name));
            break;
        default:
            throw "The specified format isn't understood";
        }
    }
    else if (op_space == 'j'){
        state_controller = 's';
        switch (controller){
        case 'p':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jp_data_type,  _js_data_type>(interface_name));
            break;
        case 's':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jp_data_type,  _js_data_type>(interface_name));
            break;
        case 'r':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jr_data_type,  _js_data_type>(interface_name));
            cmd_controller = 'p';
            break;
        case 'v':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jv_data_type,  _js_data_type>(interface_name));
            break;
        case 'f':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jf_data_type,  _js_data_type>(interface_name));
            break;
        default:
            throw "The specified format isn't understood";
        }

    }
    std::string robot_state_search_str = "measured_";
    robot_state_search_str += op_space;
    robot_state_search_str += state_controller;
    std::string robot_cmd_search_str = "servo_";
    robot_cmd_search_str += op_space;
    robot_cmd_search_str += cmd_controller;

    ctrlrBase->robot_cmd_method = rCmdIfc->get_method_by_name(robot_cmd_search_str);
    ctrlrBase->robot_state_method = rStateIfc->get_method_by_name(robot_state_search_str);

    return ctrlrBase;
}



///////
/// \brief The Interpolate class
///
class Interpolate{
public:
    Interpolate(RobotCmdIfcConstPtr rCmdIfc, RobotStateIfcConstPtr rStateIfc);
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
    CtrlrBasePtr cpCtrl;
    CtrlrBasePtr crCtrl;
    CtrlrBasePtr cvCtrl;
    CtrlrBasePtr cfCtrl;
    CtrlrBasePtr jpCtrl;
    CtrlrBasePtr jrCtrl;
    CtrlrBasePtr jvCtrl;
    CtrlrBasePtr jfCtrl;

    ControllerDataIfc ctrlrIfc;

    void execute();
    boost::thread execTh;
    vector<CtrlrBasePtr> vCtrlrs;

    _method_map_type method_map;
    _method_map_type::iterator _method_iterator;
};

Interpolate::Interpolate(RobotCmdIfcConstPtr rCmdIfc,RobotStateIfcConstPtr rStateIfc){
    method_map["interpolate_cp"] = FcnHandleBasePtr( new FcnHandle<_cp_data_type>(&Interpolate::interpolate_cp, this));
    method_map["interpolate_cr"] = FcnHandleBasePtr( new FcnHandle<_cr_data_type>(&Interpolate::interpolate_cr, this));
    method_map["interpolate_cv"] = FcnHandleBasePtr( new FcnHandle<_cv_data_type>(&Interpolate::interpolate_cv, this));
    method_map["interpolate_cf"] = FcnHandleBasePtr( new FcnHandle<_cf_data_type>(&Interpolate::interpolate_cf, this));

    method_map["interpolate_jp"] = FcnHandleBasePtr( new FcnHandle<_jp_data_type>(&Interpolate::interpolate_jp, this));
    method_map["interpolate_jr"] = FcnHandleBasePtr( new FcnHandle<_jr_data_type>(&Interpolate::interpolate_jr, this));
    method_map["interpolate_jv"] = FcnHandleBasePtr( new FcnHandle<_jv_data_type>(&Interpolate::interpolate_jv, this));
    method_map["interpolate_jf"] = FcnHandleBasePtr( new FcnHandle<_jf_data_type>(&Interpolate::interpolate_jf, this));

    cpCtrl = ctrlrIfc.create_controller_data_ifc("interpolate_cp", rCmdIfc, rStateIfc);
    crCtrl = ctrlrIfc.create_controller_data_ifc("interpolate_cr", rCmdIfc, rStateIfc);
    cvCtrl = ctrlrIfc.create_controller_data_ifc("interpolate_cv", rCmdIfc, rStateIfc);
    cfCtrl = ctrlrIfc.create_controller_data_ifc("interpolate_cf", rCmdIfc, rStateIfc);
    jpCtrl = ctrlrIfc.create_controller_data_ifc("interpolate_jp", rCmdIfc, rStateIfc);
    jrCtrl = ctrlrIfc.create_controller_data_ifc("interpolate_jr", rCmdIfc, rStateIfc);
    jvCtrl = ctrlrIfc.create_controller_data_ifc("interpolate_jv", rCmdIfc, rStateIfc);
    jfCtrl = ctrlrIfc.create_controller_data_ifc("interpolate_jf", rCmdIfc, rStateIfc);

    execTh = boost::thread(boost::bind(&Interpolate::execute, this));
}

void Interpolate::interpolate_cp(_cp_data_type &data){
    std::cout << "Called: " << __FUNCTION__ << std::endl;
    _cp_data_type robot_state;
    (*cpCtrl->robot_state_method)(robot_state);
    double t0 = time(NULL);
    double tf = t0 + 1.0;
    LinearizedPVA d0 = *cpCtrl->serialize(robot_state);
    LinearizedPVA df = *cpCtrl->serialize(data);
    cpCtrl->interpolater.compute_interpolation_params(d0.x,
                                                      df.x,
                                                      d0.dx,
                                                      df.dx,
                                                      d0.ddx,
                                                      df.ddx,
                                                      t0,
                                                      tf);
    cout << "Passed Data is \n" << df.x << std::endl;
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
    LinearizedPVA df = *jpCtrl->serialize(data);
    cout << "Passed Data is \n" << df.x << std::endl;
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



////////
/// \brief The Move class
///
class Move{
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

Move::Move(){
    method_map["move_cp"] = FcnHandleBasePtr(new FcnHandle<_cp_data_type>(&Move::move_cp, this));
    method_map["move_cr"] = FcnHandleBasePtr(new FcnHandle<_cr_data_type>(&Move::move_cr, this));
    method_map["move_jp"] = FcnHandleBasePtr(new FcnHandle<_jp_data_type>(&Move::move_jp, this));
    method_map["move_jr"] = FcnHandleBasePtr(new FcnHandle<_jr_data_type>(&Move::move_jr, this));
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

/////////////////
/// \brief The Servo class
///
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

Servo::Servo(){
    method_map["servo_cp"] = FcnHandleBasePtr(new FcnHandle<_cp_data_type>(&Servo::servo_cp, this));
    method_map["servo_cr"] = FcnHandleBasePtr(new FcnHandle<_cr_data_type>(&Servo::servo_cr, this));
    method_map["servo_cv"] = FcnHandleBasePtr(new FcnHandle<_cv_data_type>(&Servo::servo_cv, this));
    method_map["servo_cf"] = FcnHandleBasePtr(new FcnHandle<_cf_data_type>(&Servo::servo_cf, this));
    method_map["servo_jp"] = FcnHandleBasePtr(new FcnHandle<_jp_data_type>(&Servo::servo_jp, this));
    method_map["servo_jr"] = FcnHandleBasePtr(new FcnHandle<_jr_data_type>(&Servo::servo_jr, this));
    method_map["servo_jv"] = FcnHandleBasePtr(new FcnHandle<_jv_data_type>(&Servo::servo_jv, this));
    method_map["servo_jf"] = FcnHandleBasePtr(new FcnHandle<_jf_data_type>(&Servo::servo_jf, this));
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


//////////
/// \brief The Controllers class
///
class Controllers{
public:
    Controllers();
    RobotCmdIfcPtr rCmdIfc;
    RobotStateIfcPtr rStateIfc;
    boost::shared_ptr<Interpolate> interpolateCtrl;
    boost::shared_ptr<Move> moveCtrl;
    boost::shared_ptr<Servo> servoCtrl;

    FcnHandleBasePtr get_method_by_name(std::string method_name){
        std::vector<std::string> x = split_str(method_name, '_');
        std::string mode = x[0];
        char op_space = x[1][0];
        char controller = x[1][1];

        if (mode.compare("interpolate") == 0){
            return interpolateCtrl->get_method_by_name(method_name);
        }
        if (mode.compare("move") == 0){
            return moveCtrl->get_method_by_name(method_name);
        }
        if (mode.compare("servo") == 0){
            return servoCtrl->get_method_by_name(method_name);
        }
    }

private:
};

Controllers::Controllers(){
    rCmdIfc = RobotCmdIfcPtr(new RobotCmdIfc());
    rStateIfc = RobotStateIfcPtr(new RobotStateIfc());
    interpolateCtrl = boost::shared_ptr<Interpolate>(new Interpolate(rCmdIfc, rStateIfc));
    moveCtrl = boost::shared_ptr<Move>(new Move);
    servoCtrl = boost::shared_ptr<Servo>(new Servo);
}


#endif // CONTROLLERS_H
