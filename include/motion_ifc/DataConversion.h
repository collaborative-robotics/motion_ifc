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

DataConversion::DataConversion(){

}

template<>
void DataConversion::serialize<sensor_msgs::JointState>(sensor_msgs::JointState &data){
    x.resize(data.position.size());
    for (int i = 0 ; i < data.position.size(); i++){
        x[i] = data.position[i];
    }
}

template<>
void DataConversion::serialize<std::vector<double> >(std::vector<double> &data){
    x.resize(data.size());
    for (int i = 0 ; i < data.size(); i++){
        x[i] = data[i];
    }
}

template<>
void DataConversion::serialize<geometry_msgs::TransformStamped>(geometry_msgs::TransformStamped &data){
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
void DataConversion::serialize<geometry_msgs::PoseStamped>(geometry_msgs::PoseStamped &data){
    tf::Quaternion quat;
    tf::Matrix3x3 mat;
    tf::quaternionMsgToTF(data.pose.orientation, quat);
    mat.setRotation(quat);
    double r, p, y;
    mat.getRPY(r, p, y);
    x.resize(6);
    x << data.pose.position.x,
         data.pose.position.y,
         data.pose.position.z,
         r,
         p,
         y;
}

template<>
void DataConversion::serialize<geometry_msgs::Pose>(geometry_msgs::Pose &data){
    tf::Quaternion quat;
    tf::Matrix3x3 mat;
    tf::quaternionMsgToTF(data.orientation, quat);
    mat.setRotation(quat);
    double r, p, y;
    mat.getRPY(r, p, y);
    x.resize(6);
    x << data.position.x,
         data.position.y,
         data.position.z,
         r,
         p,
         y;
}

template<>
void DataConversion::deserialize<sensor_msgs::JointState>(sensor_msgs::JointState *data){
    data->position.resize(x.rows());
    for (int i = 0 ; i < x.rows(); i++){
        data->position[i] = x[i];
    }
}

template<>
void DataConversion::deserialize<std::vector<double> >(std::vector<double> *data){
    data->resize(x.rows());
    for (int i = 0 ; i < x.rows(); i++){
        data->at(i) = x[i];
    }
}

template<>
void DataConversion::deserialize<geometry_msgs::TransformStamped>(geometry_msgs::TransformStamped *data){
    data->transform.translation.x = x[0];
    data->transform.translation.y = x[1];
    data->transform.translation.z = x[2];
    tf::Quaternion quat;
    quat.setRPY(x[3], x[4], x[5]);
    tf::quaternionTFToMsg(quat, data->transform.rotation);
}

template<>
void DataConversion::deserialize<geometry_msgs::PoseStamped>(geometry_msgs::PoseStamped *data){
    data->pose.position.x = x[0];
    data->pose.position.y = x[1];
    data->pose.position.z = x[2];
    tf::Quaternion quat;
    quat.setRPY(x[3], x[4], x[5]);
    tf::quaternionTFToMsg(quat, data->pose.orientation);
}

template<>
void DataConversion::deserialize<geometry_msgs::Pose>(geometry_msgs::Pose *data){
    data->position.x = x[0];
    data->position.y = x[1];
    data->position.z = x[2];
    tf::Quaternion quat;
    quat.setRPY(x[3], x[4], x[5]);
    tf::quaternionTFToMsg(quat, data->orientation);
}



#endif
