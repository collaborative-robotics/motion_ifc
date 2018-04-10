import rospy
import numpy as np
from watch_dog import WatchDog
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import enum


class Mode(enum.Enum):
    interpolate = 0
    servo = 1
    move = 2


class OpSpace(enum.Enum):
    cartesian = 0
    joint = 1


class Controller(enum.Enum):
    position = 0
    relative = 1
    velocity = 2
    effort = 3


# Define the control Mode as a class for uniformity in the rest of the code
class ControlMode(object):
    # We are segmenting the mode as <mode>_<op_space><controller>
    # Thus using this class, we can always check back for what mode
    # does the topic specify
    mode = {}
    op_space = {}
    controller = {}
    # Three possible modes, interp, servo and move
    mode['interpolate'] = Mode.interpolate
    mode['servo'] = Mode.servo
    mode['move'] = Mode.move
    # Two possible configuration spaces, cartesian and joint
    op_space['c'] = OpSpace.cartesian
    op_space['j'] = OpSpace.joint
    # Four possible controllers, position, relative, velocity and effort
    controller['p'] = Controller.position
    controller['r'] = Controller.relative
    controller['v'] = Controller.velocity
    controller['f'] = Controller.effort


def interpret_mode_from_topic(topic_str):
    # First split to the topic name by '/'
    parsed_str = topic_str.split('/')
    # The last element should be the control mode (Maybe?)
    control_mode_str = parsed_str[-1]
    # Split the control mode string by '_' now
    control_mode_str = control_mode_str.split('_')
    # Check for format now, throw exception if the mode is not what we discussed in crtk API
    if control_mode_str.__len__() < 2:
        raise Exception('Failed to parse topic string to mode, '
                        'format should be <mode>_<op_space><controller>. E.g interp_cp')
    if control_mode_str[1].__len__() < 2:
        raise Exception('Failed to parse topic string to mode, '
                        'format should be <mode>_<op_space><controller>. E.g interp_cp')
    # Now get the relevant data for figuring out the correct mode from the control mode string
    mode_str = control_mode_str[0]
    op_space_str = control_mode_str[1][0]
    controller_str = control_mode_str[1][1]
    # Return the ControlMode now, naturally this would throw an exception if we aren't using the crtk API names
    return [ControlMode.mode[mode_str], ControlMode.op_space[op_space_str], ControlMode.controller[controller_str]]


class CommIfc(object):
    def __init__(self, topic_name, data_type, enable_wd=False, queue_size=10):
        self.topic_name = topic_name
        self.cmd = data_type()
        self.watch_dog = WatchDog
        self.enable_wd = enable_wd
        self.interpreted_mode = interpret_mode_from_topic(topic_name)

        self.sub = rospy.Subscriber(topic_name, data_type, self.message_cb, queue_size=queue_size)

    def message_cb(self, data):
        self.cmd = data
        if self.enable_wd:
            self.watch_dog.acknowledge_wd()

    def is_active(self, data):
        return self.watch_dog.is_wd_expired()

    def get_interpreted_mode(self):
        return self.interpreted_mode



class CommIfcHandler(object):
    def __init__(self):
        prefix = '/motion_ifc/'
        self.comm_ifc_list = [
            CommIfc(prefix + 'interpolate_cp', TransformStamped, True,  10),
            CommIfc(prefix + 'interpolate_cr', TransformStamped, True,  10),
            CommIfc(prefix + 'interpolate_cv', TransformStamped, True,  10),
            CommIfc(prefix + 'interpolate_cf', TransformStamped, True,  10),
            CommIfc(prefix + 'interpolate_jp', JointState,       True,  10),
            CommIfc(prefix + 'interpolate_jr', JointState,       True,  10),
            CommIfc(prefix + 'interpolate_jv', JointState,       True,  10),
            CommIfc(prefix + 'interpolate_jf', JointState,       True,  10),
            CommIfc(prefix + 'servo_cp',       TransformStamped, False, 10),
            CommIfc(prefix + 'servo_cr',       TransformStamped, False, 10),
            CommIfc(prefix + 'servo_cv',       TransformStamped, False, 10),
            CommIfc(prefix + 'servo_cf',       TransformStamped, False, 10),
            CommIfc(prefix + 'servo_jp',       JointState,       False, 10),
            CommIfc(prefix + 'servo_jr',       JointState,       False, 10),
            CommIfc(prefix + 'servo_jv',       JointState,       False, 10),
            CommIfc(prefix + 'servo_jf',       JointState,       False, 10),
            CommIfc(prefix + 'move_cp',        JointState,       False, 10),
            CommIfc(prefix + 'move_cr',        JointState,       False, 10),
            CommIfc(prefix + 'move_jp',        JointState,       False, 10),
            CommIfc(prefix + 'move_jr',        JointState,       False, 10)
        ]


class CommLogic(CommIfcHandler):
    def __init__(self):
        self._n_handle = rospy.init_node("motion_interface")
        super(CommLogic, self).__init__()
        self._rate = rospy.Rate(10)
        self._counter = 0

    def run(self):
        while not rospy.is_shutdown():
            self._rate.sleep()
            print 'Running: i = {}'.format(self._counter)
            self._counter = self._counter + 1
            pass
        self._clean()

    def _clean(self):
        for ifc in self.comm_ifc_list:
            ifc.sub.unregister()
