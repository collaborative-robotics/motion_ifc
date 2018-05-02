import rospy
import numpy as np
from watch_dog import WatchDog
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from crkt_common import *
from controllers import Controllers
from robot_state_ifc import RobotStateIfc
from communication_ifc import CommunicationIfc


class CmdCommIfc(CommunicationIfc):
    def __init__(self, topic_name, data_type, controller_ifc, feedback_ifc, enable_wd=False, queue_size=10):
        super(CmdCommIfc, self).__init__(topic_name, data_type, enable_wd, queue_size)
        self.control_mode = interpret_mode_from_topic(topic_name)
        if self.control_mode[0] is ControlMode.Mode.interpolate:
            self.wd_time_out = 0.1
        elif self.control_mode[0] is ControlMode.Mode.move:
            self.wd_time_out = 10.0
        elif self.control_mode[0] is ControlMode.Mode.servo:
            self.wd_time_out = 0.01
        else:
            raise Exception('Failed to find the right mode from topic')
        self.control_method = controller_ifc.get_method_by_name(self.get_crtk_name())
        # New we are connecting the state methods using the naming from the crtk_name of cmd
        # There is no crtk_name for relative 'r' feedback as of yet, so if the command is relative, always bind the
        # position 'p' state method
        # Additionally, there is not 'p', 'v', 'f' or 'r' for joint space feedback, we only have joint states,
        # so if that's the command always bind to 's' for op_space
        op_space_char = self.get_opspace_char()
        controller_char = self.get_controller_char()
        if controller_char is 'r':
            controller_char = 'p'
        if op_space_char is 'j':
            controller_char = 's'
        state_method_name = 'measured_' + op_space_char + controller_char
        self.state_method = feedback_ifc.get_method_by_name(state_method_name)
        self.watch_dog = WatchDog(self.wd_time_out)
        self.sub = rospy.Subscriber(topic_name, data_type, self.message_cb, queue_size=queue_size)

    def get_control_mode(self):
        return self.control_mode


class RobotCmdIfc(object):
    def __init__(self):
        self.controllers_ifc = Controllers()
        self.robot_state_ifc = RobotStateIfc()
        self.feedback_ifc = self.robot_state_ifc.feedback
        prefix = '/motion_ifc/'
        self.comm_ifc_list = [
            CmdCommIfc(prefix + 'interpolate_cp', TransformStamped, self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'interpolate_cr', TransformStamped, self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'interpolate_cv', TransformStamped, self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'interpolate_cf', TransformStamped, self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'interpolate_jp', JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'interpolate_jr', JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'interpolate_jv', JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'interpolate_jf', JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'servo_cp',       TransformStamped, self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'servo_cr',       TransformStamped, self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'servo_cv',       TransformStamped, self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'servo_cf',       TransformStamped, self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'servo_jp',       JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'servo_jr',       JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'servo_jv',       JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'servo_jf',       JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'move_cp',        JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'move_cr',        JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'move_jp',        JointState,       self.controllers_ifc, self.feedback_ifc, True, 10),
            CmdCommIfc(prefix + 'move_jr',        JointState,       self.controllers_ifc, self.feedback_ifc, True, 10)
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
            ifc.sub.unregister()
