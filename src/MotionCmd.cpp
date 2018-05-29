#include <motion_ifc/MotionCmd.h>


/////////
/// \brief MotoinCmd::MotoinCmd
///
MotionCmd::MotionCmd(){
    std::string prefix = "/motion_ifc/" + crtk::get_arm_name() + "/";
    create_motion_command_interface(prefix + "interpolate_cp", interpolate_cp_ifc, 1.0);
    create_motion_command_interface(prefix + "interpolate_cr", interpolate_cr_ifc, 1.0);
    create_motion_command_interface(prefix + "interpolate_cv", interpolate_cv_ifc, 1.0);
    create_motion_command_interface(prefix + "interpolate_cf", interpolate_cf_ifc, 1.0);

    create_motion_command_interface(prefix + "interpolate_jp", interpolate_jp_ifc, 1.0);
    create_motion_command_interface(prefix + "interpolate_jr", interpolate_jr_ifc, 1.0);
    create_motion_command_interface(prefix + "interpolate_jv", interpolate_jv_ifc, 1.0);
    create_motion_command_interface(prefix + "interpolate_jf", interpolate_jf_ifc, 1.0);

    create_motion_command_interface(prefix + "move_cp", move_cp_ifc, 5.0);
    create_motion_command_interface(prefix + "move_cr", move_cr_ifc, 5.0);
    create_motion_command_interface(prefix + "move_jp", move_jp_ifc, 5.0);
    create_motion_command_interface(prefix + "move_jr", move_jr_ifc, 5.0);

}

///
/// \brief MotoinCmd::create_motion_command_interface
/// \param topic_name
/// \param comBase
/// \param wd_timeout
///
void MotionCmd::create_motion_command_interface(string topic_name, CommBasePtr& comBase, double wd_timeout){
    comBase = commIfc.create_communication_interface(topic_name, INCOMING, wd_timeout);
    std::vector<std::string> x = split_str(topic_name, '/');
    comBase->command_method = controller_ifc.get_method_by_name(x.back());
    vec_interfaces.push_back(comBase);
}

///
/// \brief MotoinCmd::get_active_interfaces
/// \return
///
std::vector<CommBasePtr> MotionCmd::get_active_interfaces(){
    vec_activeInterfaces.clear();
    for (std::vector<CommBasePtr>::iterator it = vec_interfaces.begin(); it != vec_interfaces.end() ; it++){
        if ((*it)->is_active()){
            vec_activeInterfaces.push_back(*it);
        }
    }
    return vec_activeInterfaces;
}
