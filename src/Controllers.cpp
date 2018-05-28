#include "motion_ifc/Controllers.h"

////
/// \brief Interpolate::Interpolate
/// \param rCmdIfc
/// \param rStateIfc
///
Interpolate::Interpolate(RobotCmdIfcConstPtr rCmdIfc,RobotStateIfcConstPtr rStateIfc): rate(1000){
    method_map["interpolate_cp"] = FcnHandleBasePtr( new FcnHandle<_cp_data_type>(&Interpolate::interpolate_cp, this));
    method_map["interpolate_cr"] = FcnHandleBasePtr( new FcnHandle<_cr_data_type>(&Interpolate::interpolate_cr, this));
    method_map["interpolate_cv"] = FcnHandleBasePtr( new FcnHandle<_cv_data_type>(&Interpolate::interpolate_cv, this));
    method_map["interpolate_cf"] = FcnHandleBasePtr( new FcnHandle<_cf_data_type>(&Interpolate::interpolate_cf, this));

    method_map["interpolate_jp"] = FcnHandleBasePtr( new FcnHandle<_jp_data_type>(&Interpolate::interpolate_jp, this));
    method_map["interpolate_jr"] = FcnHandleBasePtr( new FcnHandle<_jr_data_type>(&Interpolate::interpolate_jr, this));
    method_map["interpolate_jv"] = FcnHandleBasePtr( new FcnHandle<_jv_data_type>(&Interpolate::interpolate_jv, this));
    method_map["interpolate_jf"] = FcnHandleBasePtr( new FcnHandle<_jf_data_type>(&Interpolate::interpolate_jf, this));

    cpCtrl = ctrlrIfc.create_controller_data_interface("interpolate_cp", rCmdIfc, rStateIfc);
    crCtrl = ctrlrIfc.create_controller_data_interface("interpolate_cr", rCmdIfc, rStateIfc);
    cvCtrl = ctrlrIfc.create_controller_data_interface("interpolate_cv", rCmdIfc, rStateIfc);
    cfCtrl = ctrlrIfc.create_controller_data_interface("interpolate_cf", rCmdIfc, rStateIfc);
    jpCtrl = ctrlrIfc.create_controller_data_interface("interpolate_jp", rCmdIfc, rStateIfc);
    jrCtrl = ctrlrIfc.create_controller_data_interface("interpolate_jr", rCmdIfc, rStateIfc);
    jvCtrl = ctrlrIfc.create_controller_data_interface("interpolate_jv", rCmdIfc, rStateIfc);
    jfCtrl = ctrlrIfc.create_controller_data_interface("interpolate_jf", rCmdIfc, rStateIfc);

    vec_ctrlrs.push_back(cpCtrl);
    vec_ctrlrs.push_back(crCtrl);
    vec_ctrlrs.push_back(cvCtrl);
    vec_ctrlrs.push_back(cfCtrl);
    vec_ctrlrs.push_back(jpCtrl);
    vec_ctrlrs.push_back(jrCtrl);
    vec_ctrlrs.push_back(jvCtrl);
    vec_ctrlrs.push_back(jfCtrl);

    execTh = boost::thread(boost::bind(&Interpolate::execute, this));
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


///
/// \brief Interpolate::execute
///
void Interpolate::execute(){
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
}

///
/// \brief Servo::servo_cr
/// \param data
///
void Servo::servo_cr(_cr_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Servo::servo_cv
/// \param data
///
void Servo::servo_cv(_cv_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Servo::servo_cf
/// \param data
///
void Servo::servo_cf(_cf_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Servo::servo_jp
/// \param data
///
void Servo::servo_jp(_jp_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Servo::servo_jr
/// \param data
///
void Servo::servo_jr(_jr_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Servo::servo_jv
/// \param data
///
void Servo::servo_jv(_jv_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

///
/// \brief Servo::servo_jf
/// \param data
///
void Servo::servo_jf(_jf_data_type &data){
    if(DEBUG) std::cout << "Called: " << __FUNCTION__ << std::endl;
}

////
/// \brief Controllers::Controllers
///
Controllers::Controllers(){
    rCmdIfc = RobotCmdIfcPtr(new RobotCmd());
    rStateIfc = RobotStateIfcPtr(new RobotState());
    interpolateCtrl = boost::shared_ptr<Interpolate>(new Interpolate(rCmdIfc, rStateIfc));
    moveCtrl = boost::shared_ptr<Move>(new Move);
    servoCtrl = boost::shared_ptr<Servo>(new Servo);
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
        return interpolateCtrl->get_method_by_name(method_name);
    }
    if (mode.compare("move") == 0){
        return moveCtrl->get_method_by_name(method_name);
    }
    if (mode.compare("servo") == 0){
        return servoCtrl->get_method_by_name(method_name);
    }
}

