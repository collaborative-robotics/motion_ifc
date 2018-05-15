#include <iostream>
#include <cmath>
#include "motion_ifc/DataConversion.h"
#include "motion_ifc/Controllers.h"

using namespace Eigen;
using namespace std;

#define Debug 0


int main() {
//    std::cout << "Testing Interpolation Logic using Eigen \n" << std::endl;
//    RowVectorXd x0, dx0, ddx0, xf, dxf, ddxf;
//    DataConversion dataConversion;
//    geometry_msgs::TransformStamped t_stamped;
//    t_stamped.transform.translation.x = 0.0;
//    t_stamped.transform.translation.y = 1.0;
//    t_stamped.transform.translation.z = 2.0;
//    t_stamped.transform.rotation.x = 0.0;
//    t_stamped.transform.rotation.y = 0.0;
//    t_stamped.transform.rotation.z = 0.0;
//    t_stamped.transform.rotation.w = 1.0;
//    dataConversion.serialize(t_stamped);

//    std::cout << "Transform Data: \n" << dataConversion.x << std::endl;

//    geometry_msgs::TransformStamped read_t_stamped;
//    dataConversion.deserialize(&read_t_stamped);

//    std::cout << "Retrieved Transform Translation \n" << read_t_stamped.transform.translation << std::endl;
//    std::cout << "Retrieved Transform Rotation \n" << read_t_stamped.transform.rotation << std::endl;

//    sensor_msgs::JointState joint_state;
//    joint_state.position.resize(3);
//    joint_state.position[0] = 5.2;
//    joint_state.position[1] = 4.2;
//    joint_state.position[2] = -9.2;
//    dataConversion.serialize(joint_state);

//    std::cout << "Sensor Data: \n" << dataConversion.x << std::endl;

//    sensor_msgs::JointState read_joint_state;
//    dataConversion.deserialize(&read_joint_state);
//    std::cout << "Retrieved Joint State Data: \n";
//    for (int i = 0 ; i < read_joint_state.position.size() ; i++){
//    std::cout << "Joint[" << i << "]: " << read_joint_state.position[i] << std::endl;
//    }
    Interpolate intObj;
    FcnHandle<_cp_data_type> * fcn;
    fcn = (FcnHandle<_cp_data_type> *)intObj.method_map["interpolate_cp"];
    _cp_data_type data;
    data.transform.translation.x = 5.0;
    fcn->set(data);
}
