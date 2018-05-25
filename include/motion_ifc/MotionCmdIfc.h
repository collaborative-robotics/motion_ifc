#ifndef MOTIONCMDIFC_H
#define MOTIONCMDIFC_H

#include <motion_ifc/Communication.h>
#include <motion_ifc/Controllers.h>

//////////
/// \brief The MotionCmdIfc class
///
class MotionCmdIfc{
public:
    MotionCmdIfc();

    CommBasePtr interpolate_cp_ifc;
    CommBasePtr interpolate_cr_ifc;
    CommBasePtr interpolate_cv_ifc;
    CommBasePtr interpolate_cf_ifc;

    CommBasePtr interpolate_jp_ifc;
    CommBasePtr interpolate_jr_ifc;
    CommBasePtr interpolate_jv_ifc;
    CommBasePtr interpolate_jf_ifc;

    CommBasePtr move_cp_ifc;
    CommBasePtr move_cr_ifc;

    CommBasePtr move_jp_ifc;
    CommBasePtr move_jr_ifc;

    void create_motion_command_interface(std::string topic_name, CommBasePtr& comBase, double wd_timeout);
    std::vector<CommBasePtr> get_active_interfaces();


private:
    CommunicationIfc commIfc;
    Controllers controller_ifc;
    std::vector<CommBasePtr> vec_interfaces;
    std::vector<CommBasePtr> vec_activeInterfaces;
};


/////////
/// \brief MotionCmdIfc::MotionCmdIfc
///
MotionCmdIfc::MotionCmdIfc(){
    create_motion_command_interface("/motion_ifc/interpolate_cp", interpolate_cp_ifc, 1.0);
    create_motion_command_interface("/motion_ifc/interpolate_cr", interpolate_cr_ifc, 1.0);
    create_motion_command_interface("/motion_ifc/interpolate_cv", interpolate_cv_ifc, 1.0);
    create_motion_command_interface("/motion_ifc/interpolate_cf", interpolate_cf_ifc, 1.0);

    create_motion_command_interface("/motion_ifc/interpolate_jp", interpolate_jp_ifc, 1.0);
    create_motion_command_interface("/motion_ifc/interpolate_jr", interpolate_jr_ifc, 1.0);
    create_motion_command_interface("/motion_ifc/interpolate_jv", interpolate_jv_ifc, 1.0);
    create_motion_command_interface("/motion_ifc/interpolate_jf", interpolate_jf_ifc, 1.0);

    create_motion_command_interface("/motion_ifc/move_cp", move_cp_ifc, 5.0);
    create_motion_command_interface("/motion_ifc/move_cr", move_cr_ifc, 5.0);
    create_motion_command_interface("/motion_ifc/move_jp", move_jp_ifc, 5.0);
    create_motion_command_interface("/motion_ifc/move_jr", move_jr_ifc, 5.0);

}

void MotionCmdIfc::create_motion_command_interface(string topic_name, CommBasePtr& comBase, double wd_timeout){
    comBase = commIfc.create_communication_interface(topic_name, INCOMING, wd_timeout);
    std::vector<std::string> x = split_str(topic_name, '/');
    comBase->command_method = controller_ifc.get_method_by_name(x.back());
    vec_interfaces.push_back(comBase);
}

std::vector<CommBasePtr> MotionCmdIfc::get_active_interfaces(){
    vec_activeInterfaces.clear();
    for (std::vector<CommBasePtr>::iterator it = vec_interfaces.begin(); it != vec_interfaces.end() ; it++){
        if ((*it)->is_active()){
            vec_activeInterfaces.push_back(*it);
        }
    }
    return vec_activeInterfaces;
}

#endif
