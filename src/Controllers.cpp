#include "motion_ifc/Controllers.h"

///
/// \brief SubControllersCommon::_initialized
///
bool ControllerManager::_initialized = false;

///
/// \brief SubControllersCommon::SubControllersCommon
///
ControllerManager::ControllerManager():
    rate(1000),
    RobotCmd(crtk::get_name_space(), crtk::get_arm_name() ),
    RobotState(crtk::get_name_space(), crtk::get_arm_name() ){
    if (!_initialized){
        _initialized = true;
        bind_robot_io("robot_cp", cpCtrl);
        bind_robot_io("robot_cr", crCtrl);
        bind_robot_io("robot_cv", cvCtrl);
        bind_robot_io("robot_cf", cfCtrl);

        bind_robot_io("robot_jp", jpCtrl);
        bind_robot_io("robot_jr", jrCtrl);
        bind_robot_io("robot_jv", jvCtrl);
        bind_robot_io("robot_jf", jfCtrl);

        execTh = boost::thread(boost::bind(&Interpolate::execute, this));
    }
}

///
/// \brief SubControllersCommon::bind_robot_io
/// \param interface_name
/// \param ctrlrBase
///
void ControllerManager::bind_robot_io(string interface_name, CtrlrBasePtr &ctrlrBase){
    std::vector<std::string> crtk_str = split_str(interface_name, '_');
    char op_space = crtk_str[1][0];
    char controller = crtk_str[1][1];

    char cmd_controller = controller;
    char state_controller = controller;

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
            throw "The specified format isnt understood";
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

            throw "The specified format isnt understood";
        }

    }
    std::string robot_state_search_str = "measured_";
    robot_state_search_str += op_space;
    robot_state_search_str += state_controller;
    std::string robot_cmd_search_str = "servo_";
    robot_cmd_search_str += op_space;
    robot_cmd_search_str += cmd_controller;

    if(DEBUG){
        std::cout << "Robot CMD Method: " << robot_cmd_search_str << std::endl;
        std::cout << "Robot STATE Method: " << robot_state_search_str << std::endl;
    }

    ctrlrBase->robot_cmd_method = RobotCmd::get_method_by_name(robot_cmd_search_str);
    ctrlrBase->robot_state_method = RobotState::get_method_by_name(robot_state_search_str);
    ctrlrBase->robot_cmd_ifc = RobotCmd::get_interface_by_name(robot_cmd_search_str);
    ctrlrBase->robot_state_ifc = RobotState::get_interface_by_name(robot_state_search_str);

    vec_ctrlrs.push_back(ctrlrBase);

}

///
/// \brief SubControllersCommon::execute
///
void ControllerManager::execute(){
    while (ros::ok()){
        for (std::vector<CtrlrBasePtr>::iterator it = vec_ctrlrs.begin() ; it != vec_ctrlrs.end() ; it++){
            if ((*it)->is_active()){
                (*it)->set_idle();
                double t = ros::Time::now().toSec();
                while ( (*it)->interpolater.get_t0() <= t && t <= (*it)->interpolater.get_tf()){
                    t = ros::Time::now().toSec();
                    (*it)->cmd_robot(t);
                    rate.sleep();
                }
            }
        }
    }
}

///
/// \brief Interpolate::Interpolate
///
Interpolate::Interpolate(){
    method_map["interpolate_cp"] = FcnHandleBasePtr( new FcnHandle<_cp_data_type>(&Interpolate::interpolate_cp, this));
    method_map["interpolate_cr"] = FcnHandleBasePtr( new FcnHandle<_cr_data_type>(&Interpolate::interpolate_cr, this));
    method_map["interpolate_cv"] = FcnHandleBasePtr( new FcnHandle<_cv_data_type>(&Interpolate::interpolate_cv, this));
    method_map["interpolate_cf"] = FcnHandleBasePtr( new FcnHandle<_cf_data_type>(&Interpolate::interpolate_cf, this));

    method_map["interpolate_jp"] = FcnHandleBasePtr( new FcnHandle<_jp_data_type>(&Interpolate::interpolate_jp, this));
    method_map["interpolate_jr"] = FcnHandleBasePtr( new FcnHandle<_jr_data_type>(&Interpolate::interpolate_jr, this));
    method_map["interpolate_jv"] = FcnHandleBasePtr( new FcnHandle<_jv_data_type>(&Interpolate::interpolate_jv, this));
    method_map["interpolate_jf"] = FcnHandleBasePtr( new FcnHandle<_jf_data_type>(&Interpolate::interpolate_jf, this));
}

///
/// \brief Interpolate::interpolate_cp
/// \param data
///
void Interpolate::interpolate_cp(_cp_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    _cp_data_type robot_state;
    cpCtrl->robot_state_ifc->get_data(robot_state);
    double t0 = ros::Time::now().toSec();
    double tf = t0 + cpCtrl->compute_dt(t0);
    StateSpace ss0 = *cpCtrl->serialize(robot_state);
    StateSpace ssf = *cpCtrl->serialize(data);
    if(DEBUG) std::cout << "T0: " << t0 <<  "TF: " << tf << "DT: " << tf - ros::Time::now().toSec() << std::endl;
    cpCtrl->interpolater.compute_interpolation_params(ss0, ssf, t0, tf);
    cpCtrl->set_active();
}


///
/// \brief Interpolate::interpolate_cr
/// \param data
///
void Interpolate::interpolate_cr(_cr_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Interpolate::interpolate_cv
/// \param data
///
void Interpolate::interpolate_cv(_cv_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}


///
/// \brief Interpolate::interpolate_cf
/// \param data
///
void Interpolate::interpolate_cf(_cf_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}


///
/// \brief Interpolate::interpolate_jp
/// \param data
///
void Interpolate::interpolate_jp(_jp_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    _jp_data_type robot_state;
    jpCtrl->robot_state_ifc->get_data(robot_state);
    double t0 = ros::Time::now().toSec();
    double tf = t0 + jpCtrl->compute_dt(t0);
    StateSpace ss0 = *jpCtrl->serialize(robot_state);
    StateSpace ssf = *jpCtrl->serialize(data);
    if(DEBUG) std::cout << "T0: " << t0 <<  "TF: " << tf << "DT: " << tf - ros::Time::now().toSec() << std::endl;
    jpCtrl->interpolater.compute_interpolation_params(ss0, ssf, t0, tf);
    jpCtrl->set_active();
}

///
/// \brief Interpolate::interpolate_jr
/// \param data
///
void Interpolate::interpolate_jr(_jr_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}


///
/// \brief Interpolate::interpolate_jv
/// \param data
///
void Interpolate::interpolate_jv(_jv_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}


///
/// \brief Interpolate::interpolate_jf
/// \param data
///
void Interpolate::interpolate_jf(_jf_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}


/////
/// \brief Move::Move
///
Move::Move(){
    method_map["move_cp"] = FcnHandleBasePtr(new FcnHandle<_cp_data_type>(&Move::move_cp, this));
    method_map["move_cr"] = FcnHandleBasePtr(new FcnHandle<_cr_data_type>(&Move::move_cr, this));
    method_map["move_jp"] = FcnHandleBasePtr(new FcnHandle<_jp_data_type>(&Move::move_jp, this));
    method_map["move_jr"] = FcnHandleBasePtr(new FcnHandle<_jr_data_type>(&Move::move_jr, this));
}

///
/// \brief Move::move_cp
/// \param data
///
void Move::move_cp(_cp_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Move::move_cr
/// \param data
///
void Move::move_cr(_cr_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Move::move_jp
/// \param data
///
void Move::move_jp(_jp_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Move::move_jr
/// \param data
///
void Move::move_jr(_jr_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

/////
/// \brief Servo::Servo
///
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

///
/// \brief Servo::servo_cp
/// \param data
///
void Servo::servo_cp(_cp_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    cpCtrl->robot_cmd_ifc->set_data(data);
}

///
/// \brief Servo::servo_cr
/// \param data
///
void Servo::servo_cr(_cr_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    crCtrl->robot_cmd_ifc->set_data(data);
}

///
/// \brief Servo::servo_cv
/// \param data
///
void Servo::servo_cv(_cv_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    cvCtrl->robot_cmd_ifc->set_data(data);
}

///
/// \brief Servo::servo_cf
/// \param data
///
void Servo::servo_cf(_cf_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    cfCtrl->robot_cmd_ifc->set_data(data);
}

///
/// \brief Servo::servo_jp
/// \param data
///
void Servo::servo_jp(_jp_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    jpCtrl->robot_cmd_ifc->set_data(data);
}

///
/// \brief Servo::servo_jr
/// \param data
///
void Servo::servo_jr(_jr_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    jrCtrl->robot_cmd_ifc->set_data(data);
}

///
/// \brief Servo::servo_jv
/// \param data
///
void Servo::servo_jv(_jv_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    jvCtrl->robot_cmd_ifc->set_data(data);
}

///
/// \brief Servo::servo_jf
/// \param data
///
void Servo::servo_jf(_jf_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
    jfCtrl->robot_cmd_ifc->set_data(data);
}

///
/// \brief Controllers::Controllers
/// \param name_space
/// \param arm_name
///
Controllers::Controllers(){

}


///
/// \brief Controllers::get_method_by_name
/// \param method_name
/// \return
///
FcnHandleBasePtr Controllers::get_method_by_name(std::string method_name){
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
    else{
        throw "method name not understood";
    }
}

