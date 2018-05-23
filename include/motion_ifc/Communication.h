#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <motion_ifc/crtkCommon.h>
#include <ros/ros.h>
#include <motion_ifc/FcnHandle.h>
#include <memory>
#include <motion_ifc/WatchDog.h>

using namespace std;
class CommunicationIfc;

enum CommDirection{INCOMING, OUTGOING};

//////////
/// \brief The CommunicationBase class
///
class CommunicationBase: public WatchDog{
public:
    CommunicationBase(){init();}
    static void init();

    virtual void set_data(_cp_data_type&){}
    virtual void set_data(_jp_data_type&){}

    virtual void get_data(_cp_data_type&){}
    virtual void get_data(_jp_data_type&){}

    bool is_active(){
        return (!is_wd_expired());
    }

    double last_activated_time(){
        return time_stamp;
    }

    boost::shared_ptr<FcnHandleBase> command_method;
    ros::Subscriber sub;
    ros::Publisher pub;
    static boost::shared_ptr<ros::NodeHandle> node;
protected:
    double time_stamp;
private:

};

boost::shared_ptr<ros::NodeHandle> CommunicationBase::node;

void CommunicationBase::init(){
    int argc = 0;
    char ** argv;
    ros::init(argc, argv, "motion_ifc");
    node.reset(new ros::NodeHandle);
}

/////
///
///
template <typename D>
class Communication: public CommunicationBase{
public:
    Communication(string topic_name, CommDirection com_dir=INCOMING);
    virtual void set_data(D&);
    virtual void get_data(D&);
    void cb(const boost::shared_ptr<D const> &data);

private:
    D cb_data;
};

template <typename D>
Communication<D>::Communication(string topic_name, CommDirection com_dir){
    if (com_dir == OUTGOING){
        pub = CommunicationBase::node->advertise<D>(topic_name, 10);
    }
    else if(com_dir == INCOMING ){
        sub = CommunicationBase::node->subscribe(topic_name, 10, &Communication<D>::cb, this);
    }
}

template <typename D>
void Communication<D>::cb(const boost::shared_ptr<const D> &data){
    time_stamp = ros::Time::now().toSec();
    acknowledge_wd();
    std::cout << "Data Received at " << time_stamp << std::endl;
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

//////
/// \brief The CommunicationIfc class
///
class CommunicationIfc{
public:
    CommunicationIfc(){

    }
    boost::shared_ptr<CommunicationBase> create_communication_interface(std::string interface_name, CommDirection com_dir);
};

boost::shared_ptr<CommunicationBase> CommunicationIfc::create_communication_interface(std::string interface_name, CommDirection com_dir){
    std::vector<std::string> x = split_str(interface_name, '/');
    std::vector<std::string> crtk_str = split_str(x.back(), '_');
    char op_space = crtk_str[1][0];
    char controller = crtk_str[1][1];

    std::cout << interface_name << std::endl;

    boost::shared_ptr<CommunicationBase> commBase;
    if (op_space == 'c'){
        switch (controller){
        case 'p':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_cp_data_type>(interface_name, com_dir));
            break;
        case 'r':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_cr_data_type>(interface_name, com_dir));
            break;
        case 'v':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_cv_data_type>(interface_name, com_dir));
            break;
        case 'f':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_cf_data_type>(interface_name, com_dir));
            break;
        default:
            throw "The specified format isn't understood";
        }
    }
    else if (op_space == 'j'){
        switch (controller){
        case 'p':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_jp_data_type>(interface_name, com_dir));
            break;
        case 's':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_js_data_type>(interface_name, com_dir));
            break;
        case 'r':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_jr_data_type>(interface_name, com_dir));
            break;
        case 'v':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_jv_data_type>(interface_name, com_dir));
            break;
        case 'f':
            commBase = boost::shared_ptr<CommunicationBase>(new Communication<_jf_data_type>(interface_name, com_dir));
            break;
        default:
            throw "The specified format isn't understood";
        }

    }
    std::cout << interface_name << std::endl;
    return commBase;
}


#endif
