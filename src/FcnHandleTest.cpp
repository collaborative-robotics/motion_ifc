#include <motion_ifc/Controllers.h>



int main(){
    Interpolate intObj;
    FcnHandle<_cp_data_type> * fcn;
    fcn = (FcnHandle<_cp_data_type> *)intObj.method_map["interpolate_cp"];
    _cp_data_type data;
    data.transform.translation.x = 5.0;
    (*fcn)(data);

    return 0;
}
