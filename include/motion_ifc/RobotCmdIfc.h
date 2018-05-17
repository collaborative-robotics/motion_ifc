#ifndef ROBOTCMDIFC_H
#define ROBOTCMDIFC_H

#include <motion_ifc/Communication.h>

typedef boost::shared_ptr<CommunicationBase> motionIfc;

class RobotCmdIfc{
public:
    RobotCmdIfc();

    motionIfc servo_cp_ifc;
    motionIfc servo_cr_ifc;
    motionIfc servo_cv_ifc;
    motionIfc servo_cf_ifc;

    motionIfc servo_jp_ifc;
    motionIfc servo_jr_ifc;
    motionIfc servo_jv_ifc;
    motionIfc servo_jf_ifc;

private:
    CommunicationIfc commIfc;
};

RobotCmdIfc::RobotCmdIfc(){
    servo_cp_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cp", true);
    servo_cr_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cr", true);
    servo_cv_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cv", true);
    servo_cf_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cf", true);

    servo_jp_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jp", true);
    servo_jr_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jr", true);
    servo_jv_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jv", true);
    servo_jf_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jf", true);
}

#endif
