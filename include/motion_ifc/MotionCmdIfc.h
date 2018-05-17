#ifndef MOTIONCMDIFC_H
#define MOTIONCMDIFC_H

#include <motion_ifc/Communication.h>

typedef boost::shared_ptr<CommunicationBase> motionIfc;

class MotionCmdIfc{
public:
    MotionCmdIfc();

    motionIfc interpolte_cp_ifc;
    motionIfc interpolte_cr_ifc;
    motionIfc interpolte_cv_ifc;
    motionIfc interpolte_cf_ifc;

    motionIfc interpolte_jp_ifc;
    motionIfc interpolte_jr_ifc;
    motionIfc interpolte_jv_ifc;
    motionIfc interpolte_jf_ifc;

    motionIfc move_cp_ifc;
    motionIfc move_cr_ifc;

    motionIfc move_jp_ifc;
    motionIfc move_jr_ifc;


private:
    CommunicationIfc commIfc;
};

MotionCmdIfc::MotionCmdIfc(){
    interpolte_cp_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_cp", false);
    interpolte_cr_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_cr", false);
    interpolte_cv_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_cv", false);
    interpolte_cf_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_cf", false);

    interpolte_jp_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_jp", false);
    interpolte_jr_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_jr", false);
    interpolte_jv_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_jv", false);
    interpolte_jf_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_jf", false);

    move_cp_ifc       = commIfc.create_communication_interface("/motion_ifc/move_cp", false);
    move_cr_ifc       = commIfc.create_communication_interface("/motion_ifc/move_cr", false);
    move_jp_ifc       = commIfc.create_communication_interface("/motion_ifc/move_jp", false);
    move_jr_ifc       = commIfc.create_communication_interface("/motion_ifc/move_jr", false);
}

#endif
