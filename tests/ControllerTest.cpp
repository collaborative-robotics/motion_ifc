#include <motion_ifc/Controllers.h>
#include <motion_ifc/MotionCmd.h>

int main(){

    //    Controllers controllers;
    MotionCmd motionCmd;
    ros::Rate rate(1000);
    int ctr = 0;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
        ctr ++;
//        if (ctr % 1000 == 0){
//            std::cout << "Listening: # Active Interfaces = " << motionCmd.get_active_interfaces().size() << std::endl;
//        }
        if (motionCmd.get_active_interfaces().size() > 0){
            motionCmd.get_active_interfaces()[0]->execute_controller();
        }
    }

    return 0;
}
