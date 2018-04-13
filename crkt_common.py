import enum


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
