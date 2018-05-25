#include <motion_ifc/RobotCmdIfc.h>


/////
/// \brief RobotCmdIfc::RobotCmdIfc
///
RobotCmdIfc::RobotCmdIfc(){
    servo_cp_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cp", OUTGOING);
    servo_cv_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cv", OUTGOING);
    servo_cf_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cf", OUTGOING);

    servo_jp_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jp", OUTGOING);
    servo_jv_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jv", OUTGOING);
    servo_jf_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jf", OUTGOING);

    method_map["servo_cp"] = FcnHandleBasePtr(new FcnHandle<_cp_data_type>(&CommunicationBase::set_data, servo_cp_ifc.get()));
    method_map["servo_cv"] = FcnHandleBasePtr(new FcnHandle<_cv_data_type>(&CommunicationBase::set_data, servo_cv_ifc.get()));
    method_map["servo_cf"] = FcnHandleBasePtr(new FcnHandle<_cf_data_type>(&CommunicationBase::set_data, servo_cf_ifc.get()));

    method_map["servo_jp"] = FcnHandleBasePtr(new FcnHandle<_jp_data_type>(&CommunicationBase::set_data, servo_jp_ifc.get()));
    method_map["servo_jv"] = FcnHandleBasePtr(new FcnHandle<_jv_data_type>(&CommunicationBase::set_data, servo_jv_ifc.get()));
    method_map["servo_jf"] = FcnHandleBasePtr(new FcnHandle<_jf_data_type>(&CommunicationBase::set_data, servo_jf_ifc.get()));
}

///
/// \brief RobotCmdIfc::get_method_by_name
/// \param method_name
/// \return
///
FcnHandleBasePtr RobotCmdIfc::get_method_by_name(std::string method_name){
    if (method_map.find(method_name) != method_map.end()){
        return method_map[method_name];
    }
    else{
        throw "Method name not found";
    }
}
