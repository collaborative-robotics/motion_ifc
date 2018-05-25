#ifndef CONTROLLERDATA_H
#define CONTROLLERDATA_H

#include <motion_ifc/crtkCommon.h>
#include <motion_ifc/Trajectory.h>
#include <motion_ifc/RobotCmd.h>
#include <motion_ifc/RobotState.h>

///
/// \brief The ControllerDataBase struct
///
struct ControllerDataBase: public DataConversion{
public:
    ControllerDataBase(): time_out(1.0){}
    FcnHandleBasePtr robot_cmd_method;
    FcnHandleBasePtr robot_state_method;
    CommBasePtr robot_cmd_ifc;
    CommBasePtr robot_state_ifc;
    Trajectory interpolater;

    bool _is_active;

    inline void set_active(){_is_active = true;}
    inline void set_idle(){_is_active = false;}
    inline bool is_active(){return _is_active;}

    virtual void cmd_robot(double t){}

    double compute_dt(const double &cur_time){
        double dt = cur_time - last_time;
        last_time = cur_time;
        if (dt > time_out || dt < 0){dt = time_out;}
        return dt;
    }
private:
    double last_time;
    double time_out;
};


template <typename D, typename S>
///
/// \brief The ControllerData struct
///
struct ControllerData: public ControllerDataBase{
    ControllerData(std::string interface_name){_is_active = false;}
    D cmd_data;
    S state_data;
    virtual void cmd_robot(double t);
};


///
/// \brief CtrlrBasePtr
///
typedef boost::shared_ptr<ControllerDataBase> CtrlrBasePtr;

///
/// \brief The ControllerDataIfc class
///
class ControllerDataIfc{
public:
    ControllerDataIfc(){}
    CtrlrBasePtr create_controller_data_ifc(string interface_name, RobotCmdIfcConstPtr rCmdIfcPtr, RobotStateIfcConstPtr rStateIfc);
};


#endif
