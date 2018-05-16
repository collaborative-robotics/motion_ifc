#include <motion_ifc/Controllers.h>
#include <motion_ifc/crtkCommon.h>

int main(){
    Controllers controller;
    FcnHandle<_cp_data_type> * fcn_cp = controller.get_method_by_name<_cp_data_type>("interpolate_cp");
    FcnHandle<_cr_data_type> * fcn_cr = controller.get_method_by_name<_cp_data_type>("interpolate_cr");
    FcnHandle<_cv_data_type> * fcn_cv = controller.get_method_by_name<_cp_data_type>("interpolate_cv");
    FcnHandle<_cf_data_type> * fcn_cf = controller.get_method_by_name<_cp_data_type>("interpolate_cf");
    FcnHandle<_jp_data_type> * fcn_jp = controller.get_method_by_name<_jp_data_type>("interpolate_jp");
    FcnHandle<_jr_data_type> * fcn_jr = controller.get_method_by_name<_jr_data_type>("interpolate_jr");
    FcnHandle<_jv_data_type> * fcn_jv = controller.get_method_by_name<_jv_data_type>("interpolate_jv");
    FcnHandle<_jf_data_type> * fcn_jf = controller.get_method_by_name<_jf_data_type>("interpolate_jf");

    _cp_data_type cp_data;
    _jp_data_type jp_data;
    DataConversion conversionClass;
    conversionClass.x.resize(6);
    conversionClass.x << 10, 2, -30, 90, 45, 33.33;
    conversionClass.deserialize(&cp_data);
    conversionClass.x.resize(3);
    conversionClass.x << 0.0, 1.1, 2.2;
    conversionClass.deserialize(&jp_data);

    (*fcn_cp)(cp_data);
    (*fcn_cr)(cp_data);
    (*fcn_cv)(cp_data);
    (*fcn_cf)(cp_data);
    (*fcn_jp)(jp_data);
    (*fcn_jr)(jp_data);
    (*fcn_jv)(jp_data);
    (*fcn_jf)(jp_data);


    FcnHandleBase* fcnBase;
    controller.get_method_by_name<_cp_data_type>("interpolate_cp", &fcnBase);
    (*fcnBase)(cp_data);
    controller.get_method_by_name<_cr_data_type>("interpolate_cr", &fcnBase);
    (*fcnBase)(cp_data);
    controller.get_method_by_name<_cp_data_type>("move_cr", &fcnBase);
    (*fcnBase)(cp_data);
    controller.get_method_by_name<_cf_data_type>("servo_cf", &fcnBase);
    (*fcnBase)(cp_data);

    FcnHandleBase* fcnAuto;
    Interpolate interpController;
    interpController.get_method_by_name_auto_specialized("interpolate_cp", &fcnAuto);
    (*fcnAuto)(cp_data);
    interpController.get_method_by_name_auto_specialized("interpolate_jp", &fcnAuto);
    (*fcnAuto)(jp_data);

    return 0;
}

