import rospy
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from communication_ifc import CommunicationIfc


class RobotCmdCommIfc(CommunicationIfc):
    def __init__(self, topic_name, data_type, enable_wd=False, queue_size=10):
        super(RobotCmdCommIfc, self).__init__(topic_name, data_type, enable_wd, queue_size, is_publisher=True)
        self.command_robot = self.set_data


class RobotCmdIfc(object):
    def __init__(self, namespace, arm_name):
        namespace = '/dvrk'
        arm_name = '/MTMR/'
        prefix = namespace + arm_name
        self.comm_ifc_list = [
            RobotCmdCommIfc(prefix + 'servo_cp', TransformStamped, True, 10),
            RobotCmdCommIfc(prefix + 'servo_cr', TransformStamped, True, 10),
            RobotCmdCommIfc(prefix + 'servo_cv', TransformStamped, True, 10),
            RobotCmdCommIfc(prefix + 'servo_cf', TransformStamped, True, 10),
            RobotCmdCommIfc(prefix + 'servo_jp', JointState,       True, 10),
            RobotCmdCommIfc(prefix + 'servo_jv', JointState,       True, 10),
            RobotCmdCommIfc(prefix + 'servo_jf', JointState,       True, 10)
        ]
        self._methods_dict = dict()
        for cmd_ifc in self.comm_ifc_list:
            self._methods_dict[cmd_ifc.get_crtk_name()] = cmd_ifc.command_robot

    def get_method_by_name(self, method_name):
        return self._methods_dict[method_name]

    def clean(self):
        for ifc in self.comm_ifc_list:
            ifc.disconnect()
