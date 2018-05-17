#ifndef MOTIONCMDIFC_H
#define MOTIONCMDIFC_H

#include <motion_ifc/Communication.h>

template <typename D>
class MotionCommunicatonIfc: public CommunicationBase{
public:
    MotionCommunicatonIfc(string topic_name, bool is_publisher=false);
    virtual void set_data(D&){}
    virtual void get_data(D&);
    void cb(const boost::shared_ptr<D const> &data);

private:
    D cb_data;
};

template <typename D>
MotionCommunicatonIfc<D>::MotionCommunicatonIfc(string topic_name, bool is_publisher){
    if (is_publisher){
        pub = CommunicationBase::node->advertise<D>(topic_name, 10);
    }
    else{
        sub = CommunicationBase::node->subscribe(topic_name, 10, &MotionCommunicatonIfc<D>::cb, this);
    }
}

template <typename D>
void MotionCommunicatonIfc<D>::cb(const boost::shared_ptr<const D> &data){
    std::cout << "Data Received at " << ros::Time::now().toSec() << std::endl ;
    cb_data = *data;
}

template<typename D>
void MotionCommunicatonIfc<D>::get_data(D &data){
    data = cb_data;
}


///////////////////////////////////////////////////////////////////////////////////////////

class MotionCmdIfc{
public:
    MotionCmdIfc(){

    }
    boost::shared_ptr<CommunicationBase> create_communication_interface(std::string interface_name);
};

boost::shared_ptr<CommunicationBase> MotionCmdIfc::create_communication_interface(std::string interface_name){
    std::vector<std::string> x = split_str(interface_name, '/');
    std::vector<std::string> crtk_str = split_str(x.back(), '_');
    char op_space = crtk_str[1][0];
    char controller = crtk_str[1][1];

    if (op_space == 'c'){
        boost::shared_ptr<CommunicationBase> commIfc(new MotionCommunicatonIfc<_cp_data_type>(interface_name));
        return commIfc;
    }
    else if (op_space == 'j'){
        boost::shared_ptr<CommunicationBase> commIfc(new MotionCommunicatonIfc<_jp_data_type>(interface_name));
        return commIfc;

    }
}

#endif
