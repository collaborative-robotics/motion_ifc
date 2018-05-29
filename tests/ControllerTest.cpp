#include <motion_ifc/Controllers.h>
#include <motion_ifc/MotionCmd.h>

int main(){

    //    Controllers controllers;
    crtk::set_ns_and_arm("dvrk", "MTMR");
    MotionCmd motionCmd1;
    crtk::set_ns_and_arm("dvrk", "MTML");
    MotionCmd motionCmd2;
    ros::Rate rate(1000);
    int ctr = 0;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
        ctr ++;
//        if (ctr % 1000 == 0){
//            std::cout << "Listening: # Active Interfaces = " << motionCmd.get_active_interfaces().size() << std::endl;
//        }
        if (motionCmd1.get_active_interfaces().size() > 0){
            motionCmd1.get_active_interfaces()[0]->execute_controller();
        }
        if (motionCmd2.get_active_interfaces().size() > 0){
            motionCmd2.get_active_interfaces()[0]->execute_controller();
        }
    }

    return 0;
}
