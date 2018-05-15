#ifndef DATA_CONVERSION_H
#define DATA_CONVERSION_H

#include <eigen3/Eigen/Dense>
#include <tf_conversions/tf_kdl.h>
#include <motion_ifc/crtkCommon.h>

using namespace Eigen;

struct Data{
    VectorXd x, dx, ddx;
};

class DataConversion: public Data{
public:
    DataConversion();
    template <typename T>
    void serialize(T &data);
    template <typename T>
    void deserialize(T *data);
private:
};


//template <typename T>
DataConversion::DataConversion(){

}

template<>
void DataConversion::serialize<sensor_msgs::JointState>(_jp_data_type &data){
    x.resize(data.position.size());
    for (int i = 0 ; i < data.position.size(); i++){
        x[i] = data.position[i];
    }
}

template<>
void DataConversion::serialize<geometry_msgs::TransformStamped>(_cp_data_type &data){
    tf::Quaternion quat;
    tf::Matrix3x3 mat;
    tf::quaternionMsgToTF(data.transform.rotation, quat);
    mat.setRotation(quat);
    double r, p, y;
    mat.getRPY(r, p, y);
    x.resize(6);
    x << data.transform.translation.x,
         data.transform.translation.y,
         data.transform.translation.z,
         r,
         p,
         y;
}

template<>
void DataConversion::deserialize<sensor_msgs::JointState>(_jp_data_type *data){
    data->position.resize(x.rows());
    for (int i = 0 ; i < x.rows(); i++){
        data->position[i] = x[i];
    }
}

template<>
void DataConversion::deserialize<geometry_msgs::TransformStamped>(_cp_data_type *data){
    data->transform.translation.x = x[0];
    data->transform.translation.y = x[1];
    data->transform.translation.z = x[2];
    tf::Quaternion quat;
    quat.setRPY(x[3], x[4], x[5]);
    tf::quaternionTFToMsg(quat, data->transform.rotation);
}



#endif
