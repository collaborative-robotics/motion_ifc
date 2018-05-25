#include <motion_ifc/RobotState.h>


/////
/// \brief RobotStateIfc::RobotStateIfc
///
RobotState::RobotState(){
    measured_cp_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_cp", INCOMING);
    measured_cv_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_cv", INCOMING);
    measured_cf_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_cf", INCOMING);
    measured_js_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_js", INCOMING);

    method_map["measured_cp"] = FcnHandleBasePtr( new FcnHandle<_cp_data_type>(&CommunicationBase::get_data, measured_cp_ifc.get()));
    method_map["measured_cv"] = FcnHandleBasePtr( new FcnHandle<_cv_data_type>(&CommunicationBase::get_data, measured_cv_ifc.get()));
    method_map["measured_cf"] = FcnHandleBasePtr( new FcnHandle<_cf_data_type>(&CommunicationBase::get_data, measured_cf_ifc.get()));
    method_map["measured_js"] = FcnHandleBasePtr( new FcnHandle<_js_data_type>(&CommunicationBase::get_data, measured_js_ifc.get()));

    interface_map["measured_cp"] = measured_cp_ifc;
    interface_map["measured_cv"] = measured_cv_ifc;
    interface_map["measured_cf"] = measured_cf_ifc;
    interface_map["measured_js"] = measured_js_ifc;

}

///
/// \brief get_method_by_name
/// \param method_name
/// \return
///
FcnHandleBasePtr RobotState::get_method_by_name(std::string method_name){
    if (method_map.find(method_name) != method_map.end()){
        return method_map[method_name];
    }
    else{
        throw "Method name not found";
    }
}

///
/// \brief RobotStateIfc::get_interface_by_name
/// \param interface_name
/// \return
///
CommBasePtr RobotState::get_interface_by_name(string interface_name){
    if (interface_map.find(interface_name) != interface_map.end()){
        return interface_map[interface_name];
    }
    else{
        throw "Method name not found";
    }
}
