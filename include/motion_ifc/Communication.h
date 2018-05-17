#ifndef COMMUNICATION_H
#define COMMUNICATION_H
#include <motion_ifc/crtkCommon.h>
#include <ros/ros.h>
#include <motion_ifc/FcnHandle.h>
#include <motion_ifc/Controllers.h>
#include <memory>

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

#endif
