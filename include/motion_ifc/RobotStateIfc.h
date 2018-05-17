#ifndef ROBOTSTATEIFC_H
#define ROBOTSTATEIFC_H

#include <motion_ifc/Communication.h>

typedef boost::shared_ptr<CommunicationBase> motionIfc;

class RobotStateIfc{
public:
    RobotStateIfc();

    motionIfc measured_cp_ifc;
    motionIfc measured_cv_ifc;
    motionIfc measured_cf_ifc;
    motionIfc measured_js_ifc;

private:
    CommunicationIfc commIfc;
};

RobotStateIfc::RobotStateIfc(){
    measured_cp_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_cp", false);
    measured_cv_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_cv", false);
    measured_cf_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_cf", false);
    measured_js_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_js", false);
}

#endif
