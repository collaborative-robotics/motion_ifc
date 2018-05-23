#ifndef ROBOTCMDIFC_H
#define ROBOTCMDIFC_H

#include <motion_ifc/Communication.h>

typedef std::map<std::string, boost::shared_ptr<FcnHandleBase> > _method_map_type;

class RobotCmdIfc{
public:
    RobotCmdIfc();

    inline void servo_cp(_cp_data_type &data){servo_cp_ifc->set_data(data);}
    inline void servo_cv(_cv_data_type &data){servo_cv_ifc->set_data(data);}
    inline void servo_cf(_cf_data_type &data){servo_cf_ifc->set_data(data);}

    inline void servo_jp(_jp_data_type &data){servo_jp_ifc->set_data(data);}
    inline void servo_jv(_jv_data_type &data){servo_jv_ifc->set_data(data);}
    inline void servo_jf(_jf_data_type &data){servo_jf_ifc->set_data(data);}

    comBasePtr servo_cp_ifc;
    comBasePtr servo_cr_ifc;
    comBasePtr servo_cv_ifc;
    comBasePtr servo_cf_ifc;

    comBasePtr servo_jp_ifc;
    comBasePtr servo_jr_ifc;
    comBasePtr servo_jv_ifc;
    comBasePtr servo_jf_ifc;

    boost::shared_ptr<FcnHandleBase> get_method_by_name(std::string method_name){
        if (method_map.find(method_name) != method_map.end()){
            return method_map[method_name];
        }
        else{
            throw "Method name not found";
        }
    }

    _method_map_type* get_method_names_map(){
        return &method_map;
    }


private:
    CommunicationIfc commIfc;
    _method_map_type method_map;
};

RobotCmdIfc::RobotCmdIfc(){
    servo_cp_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cp", true);
    servo_cv_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cv", true);
    servo_cf_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_cf", true);

    servo_jp_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jp", true);
    servo_jv_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jv", true);
    servo_jf_ifc = commIfc.create_communication_interface("/dvrk/MTMR/servo_jf", true);

    method_map["servo_cp"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_cp_data_type>(&CommunicationBase::set_data, servo_cp_ifc.get()));
    method_map["servo_cv"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_cv_data_type>(&CommunicationBase::set_data, servo_cv_ifc.get()));
    method_map["servo_cf"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_cf_data_type>(&CommunicationBase::set_data, servo_cf_ifc.get()));

    method_map["servo_jp"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_jp_data_type>(&CommunicationBase::set_data, servo_jp_ifc.get()));
    method_map["servo_jv"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_jv_data_type>(&CommunicationBase::set_data, servo_jv_ifc.get()));
    method_map["servo_jf"] = boost::shared_ptr<FcnHandleBase>(new FcnHandle<_jf_data_type>(&CommunicationBase::set_data, servo_jf_ifc.get()));
}

#endif
