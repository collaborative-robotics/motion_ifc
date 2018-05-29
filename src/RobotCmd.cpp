#include <motion_ifc/RobotCmd.h>


/////
/// \brief RobotCmd::RobotCmd
///
RobotCmd::RobotCmd(const string name_space, const string arm_name){
    std::string prefix = "/" + name_space + "/" + arm_name+ "/";
    servo_cp_ifc = commIfc.create_communication_interface(prefix + "servo_cp", OUTGOING);
    servo_cv_ifc = commIfc.create_communication_interface(prefix + "servo_cv", OUTGOING);
    servo_cf_ifc = commIfc.create_communication_interface(prefix + "servo_cf", OUTGOING);

    servo_jp_ifc = commIfc.create_communication_interface(prefix + "servo_jp", OUTGOING);
    servo_jv_ifc = commIfc.create_communication_interface(prefix + "servo_jv", OUTGOING);
    servo_jf_ifc = commIfc.create_communication_interface(prefix + "servo_jf", OUTGOING);

    method_map["servo_cp"] = FcnHandleBasePtr(new FcnHandle<_cp_data_type>(&CommunicationBase::set_data, servo_cp_ifc.get()));
    method_map["servo_cv"] = FcnHandleBasePtr(new FcnHandle<_cv_data_type>(&CommunicationBase::set_data, servo_cv_ifc.get()));
    method_map["servo_cf"] = FcnHandleBasePtr(new FcnHandle<_cf_data_type>(&CommunicationBase::set_data, servo_cf_ifc.get()));

    method_map["servo_jp"] = FcnHandleBasePtr(new FcnHandle<_jp_data_type>(&CommunicationBase::set_data, servo_jp_ifc.get()));
    method_map["servo_jv"] = FcnHandleBasePtr(new FcnHandle<_jv_data_type>(&CommunicationBase::set_data, servo_jv_ifc.get()));
    method_map["servo_jf"] = FcnHandleBasePtr(new FcnHandle<_jf_data_type>(&CommunicationBase::set_data, servo_jf_ifc.get()));

    interface_map["servo_cp"] = servo_cp_ifc;
    interface_map["servo_cv"] = servo_cv_ifc;
    interface_map["servo_cf"] = servo_cf_ifc;
    interface_map["servo_jp"] = servo_jp_ifc;
    interface_map["servo_jv"] = servo_jv_ifc;
    interface_map["servo_jf"] = servo_jf_ifc;
}

///
/// \brief RobotCmd::get_method_by_name
/// \param method_name
/// \return
///
FcnHandleBasePtr RobotCmd::get_method_by_name(std::string method_name){
    if (method_map.find(method_name) != method_map.end()){
        return method_map[method_name];
    }
    else{
        throw "Method name not found";
    }
}

///
/// \brief RobotCmd::get_interface_by_name
/// \param interface_name
/// \return
///
CommBasePtr RobotCmd::get_interface_by_name(string interface_name){
    if (interface_map.find(interface_name) != interface_map.end()){
        return interface_map[interface_name];
    }
    else{
        throw "Method name not found";
    }
}
