#ifndef MOTIONCMDIFC_H
#define MOTIONCMDIFC_H

#include <motion_ifc/Communication.h>

class MotionCmdIfc{
public:
    MotionCmdIfc();

    comBasePtr interpolte_cp_ifc;
    comBasePtr interpolte_cr_ifc;
    comBasePtr interpolte_cv_ifc;
    comBasePtr interpolte_cf_ifc;

    comBasePtr interpolte_jp_ifc;
    comBasePtr interpolte_jr_ifc;
    comBasePtr interpolte_jv_ifc;
    comBasePtr interpolte_jf_ifc;

    comBasePtr move_cp_ifc;
    comBasePtr move_cr_ifc;

    comBasePtr move_jp_ifc;
    comBasePtr move_jr_ifc;


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
