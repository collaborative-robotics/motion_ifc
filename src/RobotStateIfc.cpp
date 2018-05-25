#include <motion_ifc/RobotStateIfc.h>


/////
/// \brief RobotStateIfc::RobotStateIfc
///
RobotStateIfc::RobotStateIfc(){
    measured_cp_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_cp", INCOMING);
    measured_cv_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_cv", INCOMING);
    measured_cf_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_cf", INCOMING);
    measured_js_ifc = commIfc.create_communication_interface("/dvrk/MTMR/measured_js", INCOMING);

    method_map["measured_cp"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cp_data_type>(&CommunicationBase::get_data, measured_cp_ifc.get()));
    method_map["measured_cv"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cv_data_type>(&CommunicationBase::get_data, measured_cv_ifc.get()));
    method_map["measured_cf"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_cf_data_type>(&CommunicationBase::get_data, measured_cf_ifc.get()));
    method_map["measured_js"] = boost::shared_ptr<FcnHandleBase>( new FcnHandle<_js_data_type>(&CommunicationBase::get_data, measured_js_ifc.get()));
}

///
/// \brief get_method_by_name
/// \param method_name
/// \return
///
FcnHandleBasePtr RobotStateIfc::get_method_by_name(std::string method_name){
    if (method_map.find(method_name) != method_map.end()){
        return method_map[method_name];
    }
    else{
        throw "Method name not found";
    }
}
