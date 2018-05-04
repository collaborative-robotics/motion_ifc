from communication_ifc import CommunicationIfc
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, TwistStamped, WrenchStamped
import rospy


class StateCommIfc(CommunicationIfc):
    def __init__(self, topic_name, data_type, robot_state_ifc, enable_wd=False, queue_size=10):
        super(StateCommIfc, self).__init__(topic_name, data_type, enable_wd, queue_size)
        self.state_update_method = robot_state_ifc.get_method_by_name(self.get_crtk_name())
        self._last_received_time = rospy.Duration.from_sec(0.0)

    def message_cb(self, data):
        self._data = data
        self._last_received_time = rospy.Time.now().to_sec()
        if self._enable_wd:
            self._watch_dog.acknowledge_wd()
        self.state_update_method(data)

# The methods in these classes are used to both set the data and get the data. If the input argument is not 'None', the
# data is set, if the input argument is None(default) then the method returns the associated stored data. Remember that
# these methods are assigned to ros callbacks and hence they are automatically called there with the corresponding data
# passed to them as arguments


class MeasuredState(object):
    def __init__(self):
        self._cp = None
        self._cv = None
        self._cf = None
        self._js = None
        self._methods_dict = dict()
        self._methods_dict['measured_cp'] = self.measured_cp
        self._methods_dict['measured_cv'] = self.measured_cv
        self._methods_dict['measured_cf'] = self.measured_cf
        self._methods_dict['measured_js'] = self.measured_js

    def measured_cp(self, data=None):
        if data is not None:
            self._cp = data
        else:
            return self._cp

    def measured_cv(self, data=None):
        if data is not None:
            self._cv = data
        else:
            return self._cv

    def measured_cf(self, data=None):
        if data is not None:
            self._cf = data
        else:
            return self._cf

    def measured_js(self, data=None):
        if data is not None:
            self._js = data
        else:
            return self._js

    def get_methods_dict(self):
        return self._methods_dict


class GoalState(object):
    def __init__(self):
        self._cp = None
        self._cv = None
        self._js = None
        self._methods_dict = dict()
        self._methods_dict['goal_cp'] = self.goal_cp
        self._methods_dict['goal_cv'] = self.goal_cv
        self._methods_dict['goal_js'] = self.goal_js

    def goal_cp(self, data=None):
        print 'IMAPOTATO3'
        if data is not None:
            self._cp = data
        else:
            return self._cp

    def goal_cv(self, data=None):
        if data is not None:
            self._cv = data
        else:
            return self._cv

    def goal_js(self, data=None):
        if data is not None:
            self._js = data
        else:
            return self._js

    def get_methods_dict(self):
        return self._methods_dict


class SetPointState(object):
    def __init__(self):
        self._cp = None
        self._cv = None
        self._cf = None
        self._js = None
        self._methods_dict = dict()
        self._methods_dict['setpoint_cp'] = self.setpoint_cp
        self._methods_dict['setpoint_cv'] = self.setpoint_cv
        self._methods_dict['setpoint_cf'] = self.setpoint_cf
        self._methods_dict['setpoint_js'] = self.setpoint_js

    def setpoint_cp(self, data=None):
        print 'IMAPOTATO2'
        if data is not None:
            self._cp = data
        else:
            return self._cp

    def setpoint_cv(self, data=None):
        if data is not None:
            self._cv = data
        else:
            return self._cv

    def setpoint_cf(self, data=None):
        if data is not None:
            self._cf = data
        else:
            return self._cf

    def setpoint_js(self, data=None):
        if data is not None:
            self._js = data
        else:
            return self._js

    def get_methods_dict(self):
        return self._methods_dict


class Feedback(object):
    def __init__(self):
        self.state_class_list = [MeasuredState(), GoalState(), SetPointState()]
        self._methods_dict = dict()
        for state_class in self.state_class_list:
            self._methods_dict.update(state_class.get_methods_dict())
        pass

    def get_method_by_name(self, method_name):
        return self._methods_dict[method_name]


class RobotStateIfc(object):
    def __init__(self):
        self.feedback = Feedback()
        prefix = '/motion_ifc/'
        self.comm_ifc_list = [
            StateCommIfc(prefix + 'measured_cp', TransformStamped, self.feedback, True, 10),
            StateCommIfc(prefix + 'measured_cv', TwistStamped,     self.feedback, True, 10),
            StateCommIfc(prefix + 'measured_cf', WrenchStamped,    self.feedback, True, 10),
            StateCommIfc(prefix + 'measured_js', JointState,       self.feedback, True, 10),
            StateCommIfc(prefix + 'goal_cp',     TransformStamped, self.feedback, True, 10),
            StateCommIfc(prefix + 'goal_cv',     WrenchStamped,    self.feedback, True, 10),
            StateCommIfc(prefix + 'goal_js',     JointState,       self.feedback, True, 10),
            StateCommIfc(prefix + 'setpoint_cp', TransformStamped, self.feedback, True, 10),
            StateCommIfc(prefix + 'setpoint_cv', TwistStamped,     self.feedback, True, 10),
            StateCommIfc(prefix + 'setpoint_cf', WrenchStamped,    self.feedback, True, 10),
            StateCommIfc(prefix + 'setpoint_js', JointState,       self.feedback, True, 10),
        ]

    def clean(self):
        for ifc in self.comm_ifc_list:
            ifc.disconnect()



