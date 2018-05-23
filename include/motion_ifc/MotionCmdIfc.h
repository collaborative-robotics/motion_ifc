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

    void create_motion_command_interface(std::string topic_name, CommBasePtr& comBase);


private:
    CommunicationIfc commIfc;
    Controllers controller_ifc;
};


/////////
/// \brief MotionCmdIfc::MotionCmdIfc
///
MotionCmdIfc::MotionCmdIfc(){
    create_motion_command_interface("/motion_ifc/interpolate_cp", interpolate_cp_ifc);
    create_motion_command_interface("/motion_ifc/interpolate_cr", interpolate_cr_ifc);
    create_motion_command_interface("/motion_ifc/interpolate_cv", interpolate_cv_ifc);
    create_motion_command_interface("/motion_ifc/interpolate_cf", interpolate_cf_ifc);

    create_motion_command_interface("/motion_ifc/interpolate_jp", interpolate_jp_ifc);
    create_motion_command_interface("/motion_ifc/interpolate_jr", interpolate_jr_ifc);
    create_motion_command_interface("/motion_ifc/interpolate_jv", interpolate_jv_ifc);
    create_motion_command_interface("/motion_ifc/interpolate_jf", interpolate_jf_ifc);

    create_motion_command_interface("/motion_ifc/move_cp", move_cp_ifc);
    create_motion_command_interface("/motion_ifc/move_cr", move_cr_ifc);
    create_motion_command_interface("/motion_ifc/move_jp", move_jp_ifc);
    create_motion_command_interface("/motion_ifc/move_jr", move_jr_ifc);

}

void MotionCmdIfc::create_motion_command_interface(string topic_name, CommBasePtr& comBase){
    comBase = commIfc.create_communication_interface(topic_name, INCOMING);
    std::vector<std::string> x = split_str(topic_name, '/');
    comBase->command_method = controller_ifc.get_method_by_name(x.back());
}

#endif
