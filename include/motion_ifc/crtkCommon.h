#ifndef CRTK_COMMON_H
#define CRTK_COMMON_H
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <string>
#include <sstream>
#include <vector>
#include <iterator>
#include <string>

#define DEBUG 0

typedef geometry_msgs::TransformStamped _cp_data_type;
typedef geometry_msgs::TransformStamped _cr_data_type;
typedef geometry_msgs::TransformStamped _cv_data_type;
typedef geometry_msgs::TransformStamped _cf_data_type;

typedef sensor_msgs::JointState _jp_data_type;
typedef sensor_msgs::JointState _js_data_type;
typedef sensor_msgs::JointState _jr_data_type;
typedef sensor_msgs::JointState _jv_data_type;
typedef sensor_msgs::JointState _jf_data_type;

class CommunicationBase;
class FcnHandleBase;
class RobotState;
class RobotCmd;

typedef boost::shared_ptr<CommunicationBase> CommBasePtr;
typedef boost::shared_ptr<FcnHandleBase> FcnHandleBasePtr;
typedef boost::shared_ptr<RobotCmd> RobotCmdPtr;
typedef boost::shared_ptr<RobotState> RobotStatePtr;
typedef const boost::shared_ptr<RobotCmd> RobotCmdConstPtr;
typedef const boost::shared_ptr<RobotState> RobotStateConstPtr;
typedef std::map<std::string, FcnHandleBasePtr > _method_map_type;
typedef std::map<std::string, CommBasePtr > _interface_map_type;

class crtk{
public:
    crtk();
    static void set_ns_and_arm(std::string name_space, std::string arm_name);
    static std::string get_name_space();
    static std::string get_arm_name();
private:
    static std::string _name_space;
    static std::string _arm_name;
};

// Copied function from:
// https://stackoverflow.com/questions/236129/the-most-elegant-way-to-iterate-the-words-of-a-string

template<typename Out>
void split_str(const std::string &s, char delim, Out result);
std::vector<std::string> split_str(const std::string &s, char delim);


#endif
