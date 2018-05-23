#include <motion_ifc/Controllers.h>
#include <motion_ifc/MotionCmdIfc.h>

int main(){

//    Controllers controllers;
    MotionCmdIfc motionCmd;


    while(ros::ok()){
        sleep(1);
        ros::spinOnce();
        std::cout << "Listening \n";
    }


return 0;
}
