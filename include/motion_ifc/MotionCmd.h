#ifndef MOTIONCMDIFC_H
#define MOTIONCMDIFC_H

#include <motion_ifc/Communication.h>
#include <motion_ifc/Controllers.h>

//////////
/// \brief The MotionCmdIfc class
///
class MotionCmd{
public:
    MotionCmd();

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

#endif
