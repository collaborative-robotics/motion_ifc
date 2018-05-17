#include <motion_ifc/MotionCmdIfc.h>
#include <motion_ifc/DataConversion.h>


typedef boost::shared_ptr<CommunicationBase> motionIfc;
int main(){
    MotionCmdIfc commIfc;
    motionIfc interpolte_cp_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_cp");
    motionIfc interpolte_jp_ifc = commIfc.create_communication_interface("/motion_ifc/interpolate_jp");

    _cp_data_type cp_data;
    _jp_data_type jp_data;

    DataConversion dataConverter;

    while (ros::ok()){
        ros::spinOnce();
        usleep(100000);
        interpolte_cp_ifc->get_data(cp_data);
        dataConverter.serialize(cp_data);
//        std::cout << dataConverter.x << std::endl;
        interpolte_jp_ifc->get_data(jp_data);
        dataConverter.serialize(jp_data);
//        std::cout << dataConverter.x << std::endl;
    }
    return 0;
}
