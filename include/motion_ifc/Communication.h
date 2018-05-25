#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include <ros/ros.h>
#include <memory>

#include <motion_ifc/crtkCommon.h>
#include <motion_ifc/FcnHandle.h>
#include <motion_ifc/WatchDog.h>

using namespace std;
class CommunicationIfc;

enum CommDirection{INCOMING, OUTGOING};

//////////
/// \brief The CommunicationBase class
///
class CommunicationBase: public WatchDog{
public:
    CommunicationBase():_is_data_new(false){init();}
    static void init();

    virtual void set_data(_cp_data_type&){}
    virtual void set_data(_jp_data_type&){}

    virtual void get_data(_cp_data_type&){}
    virtual void get_data(_jp_data_type&){}

    inline virtual bool is_data_new(){return _is_data_new;}
    inline bool is_active(){return (!is_wd_expired());}
    inline double last_activated_time(){return time_stamp;}

    virtual void execute_controller(){}

    boost::shared_ptr<FcnHandleBase> command_method;
    ros::Subscriber sub;
    ros::Publisher pub;
    static boost::shared_ptr<ros::NodeHandle> node;
protected:
    double time_stamp;
    bool _is_data_new;
private:

};

/////
///
///
template <typename D>
class Communication: public CommunicationBase{
public:
    Communication(string topic_name, CommDirection com_dir=INCOMING, double wd_timeout=1.0);
    virtual void set_data(D&);
    virtual void get_data(D&);
    virtual void execute_controller();
    void cb(const boost::shared_ptr<D const> &data);

private:
    D cb_data;
};


//////
/// \brief The CommunicationIfc class
///
class CommunicationIfc{
public:
    CommunicationIfc(){

    }
    boost::shared_ptr<CommunicationBase> create_communication_interface(std::string topic_name, CommDirection com_dir, double wd_timeout = 1.0);
};



#endif
