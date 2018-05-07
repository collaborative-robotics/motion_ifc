import enum
from geometry_msgs.msg import TransformStamped
import geometry_msgs
from sensor_msgs.msg import JointState
import numpy as np
from tf import transformations
from PyKDL import *


# Define the control Mode as a class for uniformity in the rest of the code
class ControlMode(object):
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
    return [ControlMode.mode[mode_str],
            ControlMode.op_space[op_space_str],
            ControlMode.controller[controller_str]
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


def np_array_to_transform_stamped(array):
    if array.size >= 3:
        data = TransformStamped()
        data.transform.translation.x = array[0, 0]
        data.transform.translation.y = array[0, 1]
        data.transform.translation.z = array[0, 2]
        if array.size == 6:
            data.transform.rotation = geometry_msgs.msg.Quaternion(
                *transformations.quaternion_from_euler(array[0, 3], array[0, 4], array[0, 5]))
        return data


def np_array_to_joint_state_pos(array):
    data = JointState()
    for i in range(0, array.size):
        data.position.append(array[0, i])
    return data


def np_array_to_joint_state_vel(array):
    data = JointState()
    for i in range(0, array.size):
        data.velocity.append(array[0, i])
    return data


def np_array_to_joint_state_effort(array):
    data = JointState()
    for i in range(0, array.size):
        data.effort.append(array[0, i])
    return data


def transform_stamped_to_np_array(ts):
    rot = Rotation.Quaternion(ts.transform.rotation.x,
                              ts.transform.rotation.y,
                              ts.transform.rotation.z,
                              ts.transform.rotation.w)
    r, p, y = rot.GetRPY()
    array = np.array([ts.transform.translation.x,
                      ts.transform.translation.y,
                      ts.transform.translation.z,
                      r,
                      p,
                      y])
    return array


def transform_stamped_to_frame(ts):
    return Frame(Rotation.Quaternion(ts.transform.rotation.x,
                                     ts.transform.rotation.y,
                                     ts.transform.rotation.z,
                                     ts.transform.rotation.w),
                 Vector(ts.transform.translation.x,
                        ts.transform.translation.y,
                        ts.transform.translation.z))


def frame_to_np_array(frame):
    r, p, y = frame.M.GetRPY()
    array = np.array([frame.p[0],
                      frame.p[1],
                      frame.p[2],
                      r,
                      p,
                      y])
    return array
