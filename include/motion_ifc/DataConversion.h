#ifndef DATA_CONVERSION_H
#define DATA_CONVERSION_H

#include <eigen3/Eigen/Dense>
#include <tf_conversions/tf_kdl.h>
#include <motion_ifc/crtkCommon.h>

using namespace Eigen;

////
/// \brief The LinearizedPVA struct
///
struct LinearizedPVA{
    VectorXd x, dx, ddx;
};



///////////
/// \brief The DataConversion class
///
class DataConversion{
public:
    DataConversion();
    template <typename T>
    LinearizedPVA* serialize(T &data);
    template <typename T>
    void deserialize(T *data);
    void resize(const uint &size){
        if(pva.x.rows() != size){
            pva.x.resize(size);
            pva.dx.resize(size);
            pva.ddx.resize(size);
        }
    }
    void set_x(VectorXd &x){
        resize(x.rows());
        pva.x = x;
    }
    void set_dx(VectorXd &dx){
        resize(dx.rows());
        pva.dx = dx;
    }
    void set_ddx(VectorXd &ddx){
        resize(ddx.rows());
        pva.ddx = ddx;
    }
    VectorXd get_x(){
        return pva.x;
    }
    VectorXd get_dx(){
        return pva.dx;
    }
    VectorXd get_ddx(){
        return pva.ddx;
    }
private:
    LinearizedPVA pva;
};


DataConversion::DataConversion(){

}

template<>
LinearizedPVA* DataConversion::serialize<sensor_msgs::JointState>(sensor_msgs::JointState &data){
    resize(data.position.size());
    for (int i = 0 ; i < data.position.size(); i++){
        pva.x[i] = data.position[i];
        pva.dx[i] = 0;
        pva.ddx[i] = 0;
    }
    return &pva;
}

template<>
LinearizedPVA* DataConversion::serialize<std::vector<double> >(std::vector<double> &data){
    resize(data.size());
    for (int i = 0 ; i < data.size(); i++){
        pva.x[i] = data[i];
        pva.dx[i] = 0;
        pva.ddx[i] = 0;
    }
    return &pva;
}

template<>
LinearizedPVA* DataConversion::serialize<geometry_msgs::TransformStamped>(geometry_msgs::TransformStamped &data){
    tf::Quaternion quat;
    tf::Matrix3x3 mat;
    tf::quaternionMsgToTF(data.transform.rotation, quat);
    mat.setRotation(quat);
    double r, p, y;
    mat.getRPY(r, p, y);
    resize(6);
    pva.x << data.transform.translation.x, data.transform.translation.y, data.transform.translation.z, r, p, y;
    pva.dx << 0, 0, 0, 0, 0, 0;
    pva.ddx << 0, 0, 0, 0, 0, 0;
    return &pva;
}

template<>
LinearizedPVA* DataConversion::serialize<geometry_msgs::PoseStamped>(geometry_msgs::PoseStamped &data){
    tf::Quaternion quat;
    tf::Matrix3x3 mat;
    tf::quaternionMsgToTF(data.pose.orientation, quat);
    mat.setRotation(quat);
    double r, p, y;
    mat.getRPY(r, p, y);
    resize(6);
    pva.x << data.pose.position.x, data.pose.position.y, data.pose.position.z, r, p, y;
    pva.dx << 0, 0, 0, 0, 0, 0;
    pva.ddx << 0, 0, 0, 0, 0, 0;
    return &pva;
}

template<>
LinearizedPVA* DataConversion::serialize<geometry_msgs::Pose>(geometry_msgs::Pose &data){
    tf::Quaternion quat;
    tf::Matrix3x3 mat;
    tf::quaternionMsgToTF(data.orientation, quat);
    mat.setRotation(quat);
    double r, p, y;
    mat.getRPY(r, p, y);
    resize(6);
    pva.x << data.position.x, data.position.y, data.position.z, r, p, y;
    pva.dx << 0, 0, 0, 0, 0, 0;
    pva.ddx << 0, 0, 0, 0, 0, 0;
    return &pva;
}

template<>
void DataConversion::deserialize<sensor_msgs::JointState>(sensor_msgs::JointState *data){
    data->position.resize(pva.x.rows());
    for (int i = 0 ; i < pva.x.rows(); i++){
        data->position[i] = pva.x[i];
    }
    data->velocity.resize(pva.dx.rows());
    for (int i = 0 ; i < pva.dx.rows(); i++){
        data->velocity[i] = pva.dx[i];
    }
}

template<>
void DataConversion::deserialize<std::vector<double> >(std::vector<double> *data){
    data->resize(pva.x.rows());
    for (int i = 0 ; i < pva.x.rows(); i++){
        data->at(i) = pva.x[i];
    }
}

template<>
void DataConversion::deserialize<geometry_msgs::TransformStamped>(geometry_msgs::TransformStamped *data){
    data->transform.translation.x = pva.x[0];
    data->transform.translation.y = pva.x[1];
    data->transform.translation.z = pva.x[2];
    tf::Quaternion quat;
    quat.setRPY(pva.x[3], pva.x[4], pva.x[5]);
    tf::quaternionTFToMsg(quat, data->transform.rotation);
}

template<>
void DataConversion::deserialize<geometry_msgs::PoseStamped>(geometry_msgs::PoseStamped *data){
    data->pose.position.x = pva.x[0];
    data->pose.position.y = pva.x[1];
    data->pose.position.z = pva.x[2];
    tf::Quaternion quat;
    quat.setRPY(pva.x[3], pva.x[4], pva.x[5]);
    tf::quaternionTFToMsg(quat, data->pose.orientation);
}

template<>
void DataConversion::deserialize<geometry_msgs::Pose>(geometry_msgs::Pose *data){
    data->position.x = pva.x[0];
    data->position.y = pva.x[1];
    data->position.z = pva.x[2];
    tf::Quaternion quat;
    quat.setRPY(pva.x[3], pva.x[4], pva.x[5]);
    tf::quaternionTFToMsg(quat, data->orientation);
}



#endif
