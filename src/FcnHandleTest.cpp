#include <motion_ifc/Controllers.h>


int main(){
    Interpolate intObj;
    FcnHandle<_cp_data_type> * fcn_cp = (FcnHandle<_cp_data_type> *)intObj.get_method_by_name("interpolate_cp");
    FcnHandle<_cr_data_type> * fcn_cr = (FcnHandle<_cp_data_type> *)intObj.get_method_by_name("interpolate_cr");
    FcnHandle<_cv_data_type> * fcn_cv = (FcnHandle<_cp_data_type> *)intObj.get_method_by_name("interpolate_cv");
    FcnHandle<_cf_data_type> * fcn_cf = (FcnHandle<_cp_data_type> *)intObj.get_method_by_name("interpolate_cf");
    FcnHandle<_jp_data_type> * fcn_jp = (FcnHandle<_jp_data_type> *)intObj.get_method_by_name("interpolate_jp");
    FcnHandle<_jr_data_type> * fcn_jr = (FcnHandle<_jr_data_type> *)intObj.get_method_by_name("interpolate_jr");
    FcnHandle<_jv_data_type> * fcn_jv = (FcnHandle<_jv_data_type> *)intObj.get_method_by_name("interpolate_jv");
    FcnHandle<_jf_data_type> * fcn_jf = (FcnHandle<_jf_data_type> *)intObj.get_method_by_name("interpolate_jf");

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


    return 0;
}
