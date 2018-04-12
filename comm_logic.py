import rospy
import numpy as np
from watch_dog import WatchDog
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
import enum
from controllers import Controllers


# Define the control Mode as a class for uniformity in the rest of the code
class EnumControlMode(object):
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
    # We are segmenting the mode as <mode>_<op_space><controller>
    # Thus using this class, we can always check back for what mode
    # does the topic specify
    mode = dict()
    op_space = dict()
    controller = dict()
    # Three possible modes: interp, servo and move
    mode['interpolate'] = Mode.interpolate
    mode['servo'] = Mode.servo
    mode['move'] = Mode.move
    # Two possible configuration spaces: cartesian and joint
    op_space['c'] = OpSpace.cartesian
    op_space['j'] = OpSpace.joint
    # Four possible controllers: position, relative, velocity and effort
    controller['p'] = Controller.position
    controller['r'] = Controller.relative
    controller['v'] = Controller.velocity
    controller['f'] = Controller.effort


def interpret_mode_from_topic(topic_str):
    # First split the topic name by '/'
    parsed_str = topic_str.split('/')
    # The last element should be the control mode (Maybe? Should discuss this)
    control_mode_str = parsed_str[-1]
    # Split the control mode string by '_' to break Mode,OpSpace and Controller
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
    return [EnumControlMode.mode[mode_str],
            EnumControlMode.op_space[op_space_str],
            EnumControlMode.controller[controller_str]
            ]

def interpret_crtk_name_from_topic(topic_str):
    # First split the topic name by '/'
    parsed_str = topic_str.split('/')
    # The last element should be the control mode (Maybe? Should discuss this)
    crtk_name = parsed_str[-1]
    # Split the control mode string by '_' to break Mode,OpSpace and Controller
    control_mode_str = crtk_name.split('_')
    # Check for format now, throw exception if the mode is not what we discussed in crtk API
    if control_mode_str.__len__() < 2:
        raise Exception('Failed to parse topic string to mode, '
                        'format should be <mode>_<op_space><controller>. E.g interp_cp')
    if control_mode_str[1].__len__() < 2:
        raise Exception('Failed to parse topic string to mode, '
                        'format should be <mode>_<op_space><controller>. E.g interp_cp')
    return crtk_name


class CommIfc(object):
    def __init__(self, topic_name, data_type, controller_ifc, enable_wd=False, queue_size=10):
        self.topic_name = topic_name
        self.cmd = data_type()
        self.enable_wd = enable_wd
        self.control_mode = interpret_mode_from_topic(topic_name)
        self.crtk_name = interpret_crtk_name_from_topic(topic_name)
        self.control_method = controller_ifc.get_method_by_name(topic_name)
        self.wd_time_out = 0.0
        self.last_received_time = 0.0
        if self.control_mode[0] is EnumControlMode.Mode.interpolate:
            self.wd_time_out = 0.1
        elif self.control_mode[0] is EnumControlMode.Mode.move:
            self.wd_time_out = 10.0
        elif self.control_mode[0] is EnumControlMode.Mode.servo:
            self.wd_time_out = 0.01
        else:
            raise Exception('Failed to find the right mode from topic')
        self.watch_dog = WatchDog(self.wd_time_out)
        self.sub = rospy.Subscriber(topic_name, data_type, self.message_cb, queue_size=queue_size)

    def message_cb(self, data):
        self.cmd = data
        self.last_received_time = rospy.Time.now().to_sec()
        if self.enable_wd:
            self.watch_dog.acknowledge_wd()

    def is_active(self):
        return not self.watch_dog.is_wd_expired()

    def get_control_mode(self):
        return self.control_mode

    def get_last_received_time(self):
        return self.last_received_time

    def get_crtk_name(self):
        return self.crtk_name


class CommIfcHandler(object):
    def __init__(self):
        self.controllers_ifc = Controllers()
        prefix = '/motion_ifc/'
        self.comm_ifc_list = [
            CommIfc(prefix + 'interpolate_cp', TransformStamped, self.controllers_ifc, True,  10),
            CommIfc(prefix + 'interpolate_cr', TransformStamped, self.controllers_ifc, True,  10),
            CommIfc(prefix + 'interpolate_cv', TransformStamped, self.controllers_ifc, True,  10),
            CommIfc(prefix + 'interpolate_cf', TransformStamped, self.controllers_ifc, True,  10),
            CommIfc(prefix + 'interpolate_jp', JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'interpolate_jr', JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'interpolate_jv', JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'interpolate_jf', JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'servo_cp',       TransformStamped, self.controllers_ifc, True,  10),
            CommIfc(prefix + 'servo_cr',       TransformStamped, self.controllers_ifc, True,  10),
            CommIfc(prefix + 'servo_cv',       TransformStamped, self.controllers_ifc, True,  10),
            CommIfc(prefix + 'servo_cf',       TransformStamped, self.controllers_ifc, True,  10),
            CommIfc(prefix + 'servo_jp',       JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'servo_jr',       JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'servo_jv',       JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'servo_jf',       JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'move_cp',        JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'move_cr',        JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'move_jp',        JointState,       self.controllers_ifc, True,  10),
            CommIfc(prefix + 'move_jr',        JointState,       self.controllers_ifc, True,  10)
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
        active_ifcs_list = self.get_active_ifcs()
        last_activated_ifc = None
        lastest_time = 0.0
        for ifc in active_ifcs_list:
            ifc_time = ifc.get_last_received_time()
            if ifc_time > lastest_time:
                lastest_time = ifc_time
                last_activated_ifc = ifc
        return last_activated_ifc


class CommLogic(CommIfcHandler):
    def __init__(self):
        self._n_handle = rospy.init_node("motion_interface")
        super(CommLogic, self).__init__()
        self._rate = rospy.Rate(10)
        self._counter = 0

    def run(self):
        while not rospy.is_shutdown():
            active_ifcs_list = self.get_active_ifcs()
            last_active_ifc = self.get_last_activated_ifc()
            print 'i: {}, Number of active interfaces: {}'.format(self._counter, active_ifcs_list.__len__())
            if not last_active_ifc is None:
                print 'Lastest active ifc: {}'.format(last_active_ifc.get_crtk_name())
            if active_ifcs_list.__len__() > 0:
                ifc = active_ifcs_list[0]
                ifc.control_method()
            self._counter = self._counter + 1
            self._rate.sleep()
        self._clean()

    def _clean(self):
        for ifc in self.comm_ifc_list:
            ifc.sub.unregister()
