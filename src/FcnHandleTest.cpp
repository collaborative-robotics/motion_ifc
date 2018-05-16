#include <motion_ifc/Controllers.h>


int main(){
    Interpolate controller;
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
    cp_data.transform.translation.x = 5.0;
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
    std::cout << "Passing the same handle to separate fcn \n";
    (*fcnBase)(cp_data);
    controller.get_method_by_name<_cr_data_type>("interpolate_cr", &fcnBase);
    (*fcnBase)(cp_data);

    return 0;
}

