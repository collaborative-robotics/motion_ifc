#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <motion_ifc/crtkCommon.h>
#include <ros/ros.h>
#include <motion_ifc/FcnHandle.h>
#include <memory>

using namespace std;
class CommunicationIfc;

class CommunicationBase{

public:
    CommunicationBase(){init();}
    static void init();

    virtual void set_data(_cp_data_type&){}
    virtual void set_data(_jp_data_type&){}

    virtual void get_data(_cp_data_type&){}
    virtual void get_data(_jp_data_type&){}

    boost::shared_ptr<FcnHandleBase> command_method;
    ros::Subscriber sub;
    ros::Publisher pub;
    static boost::shared_ptr<ros::NodeHandle> node;
private:

};

boost::shared_ptr<ros::NodeHandle> CommunicationBase::node;

void CommunicationBase::init(){
    int argc = 0;
    char ** argv;
    ros::init(argc, argv, "motion_ifc");
    node.reset(new ros::NodeHandle);
}


///////////////////////////////////////////////////////////////////////////

template <typename D>
class Communication: public CommunicationBase{
public:
    Communication(string topic_name, bool is_publisher=false);
    virtual void set_data(D&);
    virtual void get_data(D&);
    void cb(const boost::shared_ptr<D const> &data);

private:
    bool is_publisher;
    D cb_data;
};

template <typename D>
Communication<D>::Communication(string topic_name, bool is_publisher){
    is_publisher = is_publisher;
    if (is_publisher){
        pub = CommunicationBase::node->advertise<D>(topic_name, 10);
    }
    else{
        sub = CommunicationBase::node->subscribe(topic_name, 10, &Communication<D>::cb, this);
    }
}

template <typename D>
void Communication<D>::cb(const boost::shared_ptr<const D> &data){
    std::cout << "Data Received at " << ros::Time::now().toSec() << std::endl ;
    cb_data = *data;
}

template<typename D>
void Communication<D>::get_data(D &data){
    data = cb_data;
}

template<typename D>
void Communication<D>::set_data(D& data){
    pub.publish(data);
}

///////////////////////////////////////////////////////////////////////////////

class CommunicationIfc{
public:
    CommunicationIfc(){

    }
    boost::shared_ptr<CommunicationBase> create_communication_interface(std::string interface_name, bool is_publisher);
};

boost::shared_ptr<CommunicationBase> CommunicationIfc::create_communication_interface(std::string interface_name, bool is_publisher){
    std::vector<std::string> x = split_str(interface_name, '/');
    std::vector<std::string> crtk_str = split_str(x.back(), '_');
    char op_space = crtk_str[1][0];
    char controller = crtk_str[1][1];

    std::cout << interface_name << std::endl;

    boost::shared_ptr<CommunicationBase> commBase;
    if (op_space == 'c'){
        switch (controller){
        case 'p':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_cp_data_type>(interface_name, is_publisher));
            break;
        case 'r':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_cr_data_type>(interface_name, is_publisher));
            break;
        case 'v':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_cv_data_type>(interface_name, is_publisher));
            break;
        case 'f':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_cf_data_type>(interface_name, is_publisher));
            break;
        default:
            throw "The specified format isn't understood";
        }
    }
    else if (op_space == 'j'){
        switch (controller){
        case 'p':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_jp_data_type>(interface_name, is_publisher));
            break;
        case 's':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_js_data_type>(interface_name, is_publisher));
            break;
        case 'r':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_jr_data_type>(interface_name, is_publisher));
            break;
        case 'v':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_jv_data_type>(interface_name, is_publisher));
            break;
        case 'f':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_jf_data_type>(interface_name, is_publisher));
            break;
        default:
            throw "The specified format isn't understood";
        }

    }
    std::cout << interface_name << std::endl;
    return commBase;
}


#endif
