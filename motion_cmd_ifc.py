import rospy
from crkt_common import *
from controllers import Controllers
from communication_ifc import CommunicationIfc


class MotionCmdCommIfc(CommunicationIfc):
    def __init__(self, topic_name, data_type, controller_ifc, enable_wd=False, queue_size=10):
        super(MotionCmdCommIfc, self).__init__(topic_name, data_type, enable_wd, queue_size)
        self.control_mode = interpret_mode_from_topic(topic_name)
        if self.control_mode[0] is ControlMode.Mode.interpolate:
            time_out = 2.0
        elif self.control_mode[0] is ControlMode.Mode.move:
            time_out = 10.0
        elif self.control_mode[0] is ControlMode.Mode.servo:
            time_out = 0.01
        else:
            raise Exception('Failed to find the right mode from topic')
        self._motion_controller_write = controller_ifc.get_method_by_name(self.get_crtk_name())
        self._watch_dog.set_timeout(time_out)

    def execute_controller(self):
        if self.is_data_new():
            self._motion_controller_write(self.get_data())

    def get_control_mode(self):
        return self.control_mode


class MotionCmdIfc(object):
    def __init__(self):
        namespace = '/dvrk'
        arm_name = '/MTMR/'
        self.ctrl = Controllers(namespace, arm_name, rospy.Time.now().to_sec())
        prefix = '/motion_ifc/'
        self.comm_ifc_list = [
            MotionCmdCommIfc(prefix + 'interpolate_cp', TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'interpolate_cr', TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'interpolate_cv', TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'interpolate_cf', TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'interpolate_jp', JointState,       self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'interpolate_jr', JointState,       self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'interpolate_jv', JointState,       self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'interpolate_jf', JointState,       self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'servo_cp',       TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'servo_cr',       TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'servo_cv',       TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'servo_cf',       TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'servo_jp',       JointState,       self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'servo_jr',       JointState,       self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'servo_jv',       JointState,       self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'servo_jf',       JointState,       self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'move_cp',        TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'move_cr',        TransformStamped, self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'move_jp',        JointState,       self.ctrl, True, 10),
            MotionCmdCommIfc(prefix + 'move_jr',        JointState,       self.ctrl, True, 10)
        ]

    # Poll through all the interfaces and check which ones are active, return a list
    # Remember, the interfaces expire based on their watchdog timeout.
    def get_active_ifcs(self):
        active_ifcs_list = []
        for ifc in self.comm_ifc_list:
            if ifc.is_active():
                active_ifcs_list.append(ifc)
        return active_ifcs_list

    def get_last_activated_ifc(self):
        last_activated_ifc = None
        lastest_time = 0.0
        for ifc in self.comm_ifc_list:
            ifc_time = ifc.get_last_received_time()
            if ifc_time > lastest_time:
                lastest_time = ifc_time
                last_activated_ifc = ifc
        return last_activated_ifc

    def clean(self):
        for ifc in self.comm_ifc_list:
            ifc.disconnect()
