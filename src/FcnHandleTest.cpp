#include <motion_ifc/Controllers.h>
#include <motion_ifc/crtkCommon.h>

int main(){
    // The easiest way of assigning data to any crtk data type is as follows

    _cp_data_type cp_data;
    _jp_data_type jp_data;
    DataConversion conversionClass;
    conversionClass.x.resize(6);
    conversionClass.x << 10, 2, -30, 90, 45, 33.33;
    conversionClass.deserialize(&cp_data);
    conversionClass.x.resize(3);
    conversionClass.x << 0.0, 1.1, 2.2;
    conversionClass.deserialize(&jp_data);

    // We can get methods from and sub-controller by passing in the controller name. The returned functor is automatically
    // specialized based on the crtk grammar

    // For Interpolate Sub-conroller
    FcnHandleBase* method_handle;
    Interpolate interpController;
    method_handle = interpController.get_method_by_name("interpolate_cp");
    (*method_handle)(cp_data);
    method_handle = interpController.get_method_by_name("interpolate_jp");
    (*method_handle)(jp_data);

    // For Move Sub-conroller
    Move moveController;
    method_handle = moveController.get_method_by_name("move_cp");
    (*method_handle)(cp_data);
    method_handle = moveController.get_method_by_name("move_jp");
    (*method_handle)(jp_data);

    // For Servo Sub-conroller
    Servo servoController;
    method_handle = servoController.get_method_by_name("servo_cp");
    (*method_handle)(cp_data);
    method_handle = servoController.get_method_by_name("servo_jp");
    (*method_handle)(jp_data);



    // And we can get and subcontroller method from the controller class as well
    Controllers controller;
    FcnHandleBase * interpolate_cp = controller.get_method_by_name("interpolate_cp");
    FcnHandleBase * interpolate_cr = controller.get_method_by_name("interpolate_cr");
    FcnHandleBase * interpolate_cv = controller.get_method_by_name("interpolate_cv");
    FcnHandleBase * interpolate_cf = controller.get_method_by_name("interpolate_cf");
    FcnHandleBase * interpolate_jp = controller.get_method_by_name("interpolate_jp");
    FcnHandleBase * interpolate_jr = controller.get_method_by_name("interpolate_jr");
    FcnHandleBase * interpolate_jv = controller.get_method_by_name("interpolate_jv");
    FcnHandleBase * interpolate_jf = controller.get_method_by_name("interpolate_jf");

    FcnHandleBase * servo_cp = controller.get_method_by_name("servo_cp");
    FcnHandleBase * servo_cr = controller.get_method_by_name("servo_cr");
    FcnHandleBase * servo_cv = controller.get_method_by_name("servo_cv");
    FcnHandleBase * servo_cf = controller.get_method_by_name("servo_cf");
    FcnHandleBase * servo_jp = controller.get_method_by_name("servo_jp");
    FcnHandleBase * servo_jr = controller.get_method_by_name("servo_jr");
    FcnHandleBase * servo_jv = controller.get_method_by_name("servo_jv");
    FcnHandleBase * servo_jf = controller.get_method_by_name("servo_jf");

    FcnHandleBase * move_cp = controller.get_method_by_name("move_cp");
    FcnHandleBase * move_cr = controller.get_method_by_name("move_cr");
    FcnHandleBase * move_jp = controller.get_method_by_name("move_jp");
    FcnHandleBase * move_jr = controller.get_method_by_name("move_jr");

    (*interpolate_cp)(cp_data);
    (*interpolate_cr)(cp_data);
    (*interpolate_cv)(cp_data);
    (*interpolate_cf)(cp_data);
    (*interpolate_jp)(jp_data);
    (*interpolate_jr)(jp_data);
    (*interpolate_jv)(jp_data);
    (*interpolate_jf)(jp_data);

    (*servo_cp)(cp_data);
    (*servo_cr)(cp_data);
    (*servo_cv)(cp_data);
    (*servo_cf)(cp_data);
    (*servo_jp)(jp_data);
    (*servo_jr)(jp_data);
    (*servo_jv)(jp_data);
    (*servo_jf)(jp_data);

    (*move_cp)(cp_data);
    (*move_cr)(cp_data);
    (*move_jp)(jp_data);
    (*move_jr)(jp_data);

    return 0;
}

