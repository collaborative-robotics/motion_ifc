#include <motion_ifc/ControllerData.h>


///
/// \brief ControllerDataIfc::create_controller_data_ifc
/// \param interface_name
/// \param rCmdIfc
/// \param rStateIfc
/// \return
///
CtrlrBasePtr ControllerDataIfc::create_controller_data_interface(string interface_name, RobotCmdConstPtr rCmdIfc, RobotStateConstPtr rStateIfc){
    std::vector<std::string> x = split_str(interface_name, '/');
    std::vector<std::string> crtk_str = split_str(x.back(), '_');
    char op_space = crtk_str[1][0];
    char controller = crtk_str[1][1];

    char cmd_controller = controller;
    char state_controller = controller;

    CtrlrBasePtr ctrlrBase;
    if (op_space == 'c'){
        switch (controller){
        case 'p':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_cp_data_type,  _cp_data_type>(interface_name));
            break;
        case 'r':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_cr_data_type,  _cr_data_type>(interface_name));
            cmd_controller = 'p';
            state_controller = 'p';
            break;
        case 'v':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_cv_data_type,  _cv_data_type>(interface_name));
            break;
        case 'f':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_cf_data_type,  _cf_data_type>(interface_name));
            break;
        default:
            throw "The specified format isnt understood";
        }
    }
    else if (op_space == 'j'){
        state_controller = 's';
        switch (controller){
        case 'p':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jp_data_type,  _js_data_type>(interface_name));
            break;
        case 's':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jp_data_type,  _js_data_type>(interface_name));
            break;
        case 'r':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jr_data_type,  _js_data_type>(interface_name));
            cmd_controller = 'p';
            break;
        case 'v':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jv_data_type,  _js_data_type>(interface_name));
            break;
        case 'f':
            ctrlrBase = CtrlrBasePtr(new ControllerData<_jf_data_type,  _js_data_type>(interface_name));
            break;
        default:
            throw "The specified format isnt understood";
        }

    }
    std::string robot_state_search_str = "measured_";
    robot_state_search_str += op_space;
    robot_state_search_str += state_controller;
    std::string robot_cmd_search_str = "servo_";
    robot_cmd_search_str += op_space;
    robot_cmd_search_str += cmd_controller;

    ctrlrBase->robot_cmd_method = rCmdIfc->get_method_by_name(robot_cmd_search_str);
    ctrlrBase->robot_state_method = rStateIfc->get_method_by_name(robot_state_search_str);
    ctrlrBase->robot_cmd_ifc = rCmdIfc->get_interface_by_name(robot_cmd_search_str);
    ctrlrBase->robot_state_ifc = rStateIfc->get_interface_by_name(robot_state_search_str);

    return ctrlrBase;
}

template<typename D, typename S>
///
/// \brief ControllerData<D, S>::cmd_robot
/// \param pva
///
void ControllerData<D, S>::cmd_robot(StateSpace &pva){
    deserialize(&cmd_data, &pva);
    robot_cmd_ifc->set_data(cmd_data);
}

template<typename D, typename S>
///
/// \brief ControllerData<D, S>::cmd_robot
/// \param t
///
void ControllerData<D, S>::cmd_robot(double t){
    StateSpace pva = interpolater.get_interpolated_state_space(t);
    cmd_robot(pva);
}

template<typename D, typename S>
///
/// \brief ControllerData<D, S>::cmd_robot
///
void ControllerData<D, S>::cmd_robot(){
    cmd_robot(ros::Time::now().toSec());
}
