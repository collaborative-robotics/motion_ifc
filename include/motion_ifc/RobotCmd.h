#ifndef ROBOTCMDIFC_H
#define ROBOTCMDIFC_H

#include <motion_ifc/Communication.h>


//////
/// \brief The RobotCmd class
///
class RobotCmd{
public:
    RobotCmd(const std::string name_space, const std::string arm_name);

    inline void servo_cp(_cp_data_type &data){servo_cp_ifc->set_data(data);}
    inline void servo_cv(_cv_data_type &data){servo_cv_ifc->set_data(data);}
    inline void servo_cf(_cf_data_type &data){servo_cf_ifc->set_data(data);}

    inline void servo_jp(_jp_data_type &data){servo_jp_ifc->set_data(data);}
    inline void servo_jv(_jv_data_type &data){servo_jv_ifc->set_data(data);}
    inline void servo_jf(_jf_data_type &data){servo_jf_ifc->set_data(data);}

    CommBasePtr servo_cp_ifc;
    CommBasePtr servo_cr_ifc;
    CommBasePtr servo_cv_ifc;
    CommBasePtr servo_cf_ifc;

    CommBasePtr servo_jp_ifc;
    CommBasePtr servo_jr_ifc;
    CommBasePtr servo_jv_ifc;
    CommBasePtr servo_jf_ifc;

    FcnHandleBasePtr get_method_by_name(std::string method_name); 
    CommBasePtr get_interface_by_name(std::string interface_name);

    inline _method_map_type* get_method_names_map(){return &method_map;}


private:
    CommunicationIfc commIfc;
    _method_map_type method_map;
    _interface_map_type interface_map;
};

#endif
