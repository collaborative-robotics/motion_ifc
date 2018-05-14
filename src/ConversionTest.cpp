#include <iostream>
#include <cmath>
#include "motion_ifc/DataConversion.h"

using namespace Eigen;
using namespace std;

#define Debug 0


int main() {
    std::cout << "Testing Interpolation Logic using Eigen \n" << std::endl;
    RowVectorXd x0, dx0, ddx0, xf, dxf, ddxf;
    DataConversion test_obj;
    geometry_msgs::TransformStamped data;
    data.transform.translation.x = 0.0;
    data.transform.translation.y = 1.0;
    data.transform.translation.z = 2.0;
    data.transform.rotation.x = 0.0;
    data.transform.rotation.y = 0.0;
    data.transform.rotation.z = 0.0;
    data.transform.rotation.w = 1.0;
    sensor_msgs::JointState jdata;
    test_obj.serialize(data);
    std::cout << "Transform Data: \n" << test_obj.x << std::endl;
    jdata.position.resize(3);
    jdata.position[0] = 5.2;
    jdata.position[1] = 4.2;
    jdata.position[2] = -9.2;
    test_obj.serialize(jdata);
    std::cout << "Sensor Data: \n" << test_obj.x << std::endl;
}
