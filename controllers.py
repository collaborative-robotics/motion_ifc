from interp_logic import Interpolation
import rospy
from threading import Thread
from crkt_common import *
from robot_cmd_ifc import RobotCmdIfc
from robot_state_ifc import RobotStateIfc


# Each sub-controller must provide a dict() of all the methods with the method
# name as the key and the method handle as the value so that this Controller
# class and append all the dict() items in a unified way


class ControllerData:
    def __init__(self, cmd_robot_method, state_robot_method):
        self.interpolater = Interpolation()
        self._active = False
        self.set_robot_command = cmd_robot_method
        self.get_robot_state = state_robot_method
        self._dt = 1.0
        self._dt_max = 5.0
        self._last_call_time = 0.0

    def set_active(self):
        self._active = True

    def is_active(self):
        return self._active

    def set_idle(self):
        self._active = False

    def set_dt_max(self, dt_max):
        if 1.0 < dt_max < 10.0:
            self._dt_max = dt_max

    def xt(self, t):
        xt = self.interpolater.get_interpolated_x(t)
        return np_array_to_transform_stamped(xt)

    def calculate_dt(self, cur_time):
        dt = cur_time - self._last_call_time
        self._last_call_time = cur_time
        if 0 < dt <= self._dt_max:
            self._dt = dt
        return self._dt


class Interpolate(object):
    def __init__(self, r_cmd, r_state, t_start):
        self._t_start = t_start
        self._methods_dict = dict()
        self._methods_dict['interpolate_cp'] = self.interpolate_cp
        self._methods_dict['interpolate_cr'] = self.interpolate_cr
        self._methods_dict['interpolate_cv'] = self.interpolate_cv
        self._methods_dict['interpolate_cf'] = self.interpolate_cf
        self._methods_dict['interpolate_jp'] = self.interpolate_jp
        self._methods_dict['interpolate_jr'] = self.interpolate_jr
        self._methods_dict['interpolate_jv'] = self.interpolate_jv
        self._methods_dict['interpolate_jf'] = self.interpolate_jf

        self._cp_ctrl = ControllerData(r_cmd.get_method_by_name('servo_cp'), r_state.get_method_by_name('measured_cp'))
        self._cr_ctrl = ControllerData(r_cmd.get_method_by_name('servo_cp'), r_state.get_method_by_name('measured_cp'))
        self._cv_ctrl = ControllerData(r_cmd.get_method_by_name('servo_cv'), r_state.get_method_by_name('measured_cv'))
        self._cf_ctrl = ControllerData(r_cmd.get_method_by_name('servo_cf'), r_state.get_method_by_name('measured_cf'))
        self._jp_ctrl = ControllerData(r_cmd.get_method_by_name('servo_jp'), r_state.get_method_by_name('measured_js'))
        self._jr_ctrl = ControllerData(r_cmd.get_method_by_name('servo_jp'), r_state.get_method_by_name('measured_js'))
        self._jv_ctrl = ControllerData(r_cmd.get_method_by_name('servo_jv'), r_state.get_method_by_name('measured_js'))
        self._jf_ctrl = ControllerData(r_cmd.get_method_by_name('servo_jf'), r_state.get_method_by_name('measured_js'))

        self._controller_data_list = [self._cp_ctrl,
                                      self._cr_ctrl,
                                      self._cv_ctrl,
                                      self._cf_ctrl,
                                      self._jp_ctrl,
                                      self._jr_ctrl,
                                      self._jv_ctrl,
                                      self._jf_ctrl]

        self._thread = Thread(target=self._execute)
        self._thread.start()

    def interpolate_cp(self, cmd):
        state = self._cp_ctrl.get_robot_state()
        if cmd is not None and state is not None:
            p0 = transform_stamped_to_np_array(state)
            pf = transform_stamped_to_np_array(cmd)
            v0 = [0, 0, 0, 0, 0, 0]
            vf = [0, 0, 0, 0, 0, 0]
            a0 = [0, 0, 0, 0, 0, 0]
            af = [0, 0, 0, 0, 0, 0]

            cur_time = rospy.Time.now().to_sec()
            t0 = cur_time - self._t_start
            tf = t0 + self._cp_ctrl.calculate_dt(t0)
            self._cp_ctrl.interpolater.compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)
            self._cp_ctrl.set_active()
        pass

    def interpolate_cr(self, cmd, state):
        pass

    def interpolate_cv(self, cmd, state):
        pass

    def interpolate_cf(self, cmd, state):
        pass

    def interpolate_jp(self, cmd, state):
        pass

    def interpolate_jr(self, cmd, state):
        pass

    def interpolate_jv(self, cmd, state):
        pass

    def interpolate_jf(self, cmd, state):
        pass

    def _execute(self):
        while not rospy.is_shutdown():
            for controller_data in self._controller_data_list:
                if controller_data.is_active():
                    t = rospy.Time.now().to_sec() - self._t_start
                    while t < controller_data.interpolater.get_tf():
                        data = controller_data.xt(t)
                        controller_data.set_robot_command(data)
                        t = rospy.Time.now().to_sec() - self._t_start
                    controller_data.set_idle()

    def get_methods_dict(self):
        return self._methods_dict


class Servo(object):
    def __init__(self, robot_cmd_ifc, t_start):
        self._methods_dict = dict()
        self._methods_dict['servo_cp'] = self.servo_cp
        self._methods_dict['servo_cr'] = self.servo_cr
        self._methods_dict['servo_cv'] = self.servo_cv
        self._methods_dict['servo_cf'] = self.servo_cf
        self._methods_dict['servo_jp'] = self.servo_jp
        self._methods_dict['servo_jr'] = self.servo_jr
        self._methods_dict['servo_jv'] = self.servo_jv
        self._methods_dict['servo_jf'] = self.servo_jf

    def servo_cp(self):
        pass

    def servo_cr(self):
        pass

    def servo_cv(self):
        pass

    def servo_cf(self):
        pass

    def servo_jp(self):
        pass

    def servo_jr(self):
        pass

    def servo_jv(self):
        pass

    def servo_jf(self):
        pass

    def get_methods_dict(self):
        return self._methods_dict


class Move(object):
    def __init__(self, robot_cmd_ifc, t_start):
        self._methods_dict = dict()
        self._methods_dict['move_cp'] = self.move_cp
        self._methods_dict['move_cr'] = self.move_cr
        self._methods_dict['move_jp'] = self.move_jp
        self._methods_dict['move_jr'] = self.move_jr

    def move_cp(self):
        pass

    def move_cr(self):
        pass

    def move_jp(self):
        pass

    def move_jr(self):
        pass

    def get_methods_dict(self):
        return self._methods_dict


class Controllers(object):
    def __init__(self, namespace, arm_name, t_start=0.0):
        self._robot_cmd_ifc = RobotCmdIfc(namespace, arm_name)
        self._robot_state_ifc = RobotStateIfc(namespace, arm_name)
        self.controllers_list = [Interpolate(self._robot_cmd_ifc, self._robot_state_ifc, t_start),
                                 Servo(self._robot_cmd_ifc, t_start),
                                 Move(self._robot_cmd_ifc, t_start)]
        self._methods_dict = dict()
        for controller in self.controllers_list:
            self._methods_dict.update(controller.get_methods_dict())
        pass

    def get_method_by_name(self, method_name):
        return self._methods_dict[method_name]

    def disconnect(self):
        self._robot_state_ifc.clean()
        self._robot_cmd_ifc.clean()


