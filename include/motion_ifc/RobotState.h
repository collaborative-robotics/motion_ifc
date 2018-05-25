#ifndef ROBOTSTATEIFC_H
#define ROBOTSTATEIFC_H

#include <motion_ifc/Communication.h>

//class Measured{
//    Measured();
//    _cp_data_type measured_cp();
//    _cv_data_type measured_cv();
//    _cf_data_type measured_cf();

//    _jp_data_type measured_jp();
//    _jp_data_type measured_js();
//    _jv_data_type measured_jv();
//    _jf_data_type measured_jf();
//};

//class Setpoint{
//    Setpoint();
//    _cp_data_type setpoint_cp();
//    _cv_data_type setpoint_cv();
//    _cf_data_type setpoint_cf();

//    _jp_data_type setpoint_jp();
//    _jr_data_type setpoint_jr();
//    _jv_data_type setpoint_jv();
//    _jf_data_type setpoint_jf();
//};

//class Goal{
//    Goal();
//    _cp_data_type goal_cp();
//    _cv_data_type goal_cv();
//    _cf_data_type goal_cf();

//    _jp_data_type goal_jp();
//    _js_data_type goal_js();
//    _jv_data_type goal_jv();
//    _jf_data_type goal_jf();
//};

/////////////////
/// \brief The RobotStateIfc class
///
class RobotState{
public:
    RobotState();

    CommBasePtr measured_cp_ifc;
    CommBasePtr measured_cv_ifc;
    CommBasePtr measured_cf_ifc;
    CommBasePtr measured_js_ifc;

    inline _cp_data_type measured_cp(){measured_cp_ifc->get_data(measured_cp_data); return measured_cp_data;}
    inline _cv_data_type measured_cv(){measured_cp_ifc->get_data(measured_cv_data); return measured_cv_data;}
    inline _cf_data_type measured_cf(){measured_cp_ifc->get_data(measured_cf_data); return measured_cf_data;}
    inline _js_data_type measured_js(){measured_js_ifc->get_data(measured_js_data); return measured_js_data;}

    inline void measured_cp(_cp_data_type &data){measured_cp_ifc->get_data(data);}
    inline void measured_cv(_cv_data_type &data){measured_cp_ifc->get_data(data);}
    inline void measured_cf(_cf_data_type &data){measured_cp_ifc->get_data(data);}
    inline void measured_js(_js_data_type &data){measured_js_ifc->get_data(data);}

//    _cp_data_type setpoint_cp(){setpoint_cp_ifc->get_data(setpoint_cp_data); return setpoint_cp_data;}
//    _cv_data_type setpoint_cv(){setpoint_cv_ifc->get_data(setpoint_cv_data); return setpoint_cv_data;}
//    _cf_data_type setpoint_cf(){setpoint_cf_ifc->get_data(setpoint_cf_data); return setpoint_cf_data;}
//    _js_data_type setpoint_js(){setpoint_cp_ifc->get_data(setpoint_js_data); return setpoint_js_data;}

//    _cp_data_type goal_cp(){goal_cp_ifc->get_data(goal_cp_data); return goal_cp_data;}
//    _cv_data_type goal_cv(){goal_cv_ifc->get_data(goal_cv_data); return goal_cv_data;}
//    _cf_data_type goal_cf(){goal_cf_ifc->get_data(goal_cf_data); return goal_cf_data;}
//    _js_data_type goal_js(){goal_cp_ifc->get_data(goal_js_data); return goal_js_data;}

    FcnHandleBasePtr get_method_by_name(std::string method_name);
    CommBasePtr get_interface_by_name(std::string interface_name);

    inline _method_map_type* get_method_names_map(){return &method_map;}


private:

    _cp_data_type measured_cp_data;
    _cv_data_type measured_cv_data;
    _cf_data_type measured_cf_data;
    _js_data_type measured_js_data;

    _cp_data_type setpoint_cp_data;
    _cv_data_type setpoint_cv_data;
    _cf_data_type setpoint_cf_data;
    _js_data_type setpoint_js_data;

    _cp_data_type goal_cp_data;
    _cv_data_type goal_cv_data;
    _cf_data_type goal_cf_data;
    _js_data_type goal_js_data;

    CommunicationIfc commIfc;
    _method_map_type method_map;
    _interface_map_type interface_map;
};


#endif
