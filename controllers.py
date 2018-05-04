from interp_logic import Interpolation
import rospy
from threading import Thread
from geometry_msgs.msg import TransformStamped


# Each sub-controller must provide a dict() of all the methods with the method
# name as the key and the method handle as the value so that this Controller
# class and append all the dict() items in a unified way


class ControllerData:
    def __init__(self, cmd_robot_method):
        self.interpolater = Interpolation()
        self._active = 0
        self.command_robot = cmd_robot_method

    def set_active(self):
        self._active = True

    def is_active(self):
        return self._active

    def set_idle(self):
        self._active = False

    def xt(self, t):
        xt = self.interpolater.get_interpolated_x(t)
        data = TransformStamped()
        data.transform.translation.x = xt[0, 0]
        data.transform.translation.y = xt[0, 1]
        data.transform.translation.z = xt[0, 2]
        return data


class Interpolate(object):
    def __init__(self, robot_cmd_ifc, t_start):
        self._t_start = t_start
        self._delta_t = 0.5
        self._methods_dict = dict()
        self._methods_dict['interpolate_cp'] = self.interpolate_cp
        self._methods_dict['interpolate_cr'] = self.interpolate_cr
        self._methods_dict['interpolate_cv'] = self.interpolate_cv
        self._methods_dict['interpolate_cf'] = self.interpolate_cf
        self._methods_dict['interpolate_jp'] = self.interpolate_jp
        self._methods_dict['interpolate_jr'] = self.interpolate_jr
        self._methods_dict['interpolate_jv'] = self.interpolate_jv
        self._methods_dict['interpolate_jf'] = self.interpolate_jf

        self._cp_controller_data = ControllerData(robot_cmd_ifc.get_method_by_name('servo_cp'))
        self._cr_controller_data = ControllerData(robot_cmd_ifc.get_method_by_name('servo_cp'))
        self._cv_controller_data = ControllerData(robot_cmd_ifc.get_method_by_name('servo_cv'))
        self._cf_controller_data = ControllerData(robot_cmd_ifc.get_method_by_name('servo_cf'))
        self._jp_controller_data = ControllerData(robot_cmd_ifc.get_method_by_name('servo_jp'))
        self._jr_controller_data = ControllerData(robot_cmd_ifc.get_method_by_name('servo_jp'))
        self._jv_controller_data = ControllerData(robot_cmd_ifc.get_method_by_name('servo_jv'))
        self._jf_controller_data = ControllerData(robot_cmd_ifc.get_method_by_name('servo_jf'))

        self._controller_data_list = [self._cp_controller_data,
                                      self._cr_controller_data,
                                      self._cv_controller_data,
                                      self._cf_controller_data,
                                      self._jp_controller_data,
                                      self._jr_controller_data,
                                      self._jv_controller_data,
                                      self._jf_controller_data]

        self._thread = Thread(target=self._execute)
        self._thread.start()

    def interpolate_cp(self, cmd, state):
        if state is not None and cmd is not None:
            p0 = [state.transform.translation.x,
                  state.transform.translation.y,
                  state.transform.translation.z]
            pf = [cmd.transform.translation.x,
                  cmd.transform.translation.y,
                  cmd.transform.translation.z]
            v0 = [0, 0, 0]
            vf = [0, 0, 0]
            a0 = [0, 0, 0]
            af = [0, 0, 0]

            t0 = rospy.Time.now().to_sec() - self._t_start
            tf = t0 + self._delta_t
            self._cp_controller_data.interpolater.compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)
            self._cp_controller_data.set_active()
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
                        controller_data.command_robot(data)
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
    def __init__(self, robot_cmd_ifc, t_start=0.0):
        self.controllers_list = [Interpolate(robot_cmd_ifc, t_start),
                                 Servo(robot_cmd_ifc, t_start),
                                 Move(robot_cmd_ifc, t_start)]
        self._methods_dict = dict()
        for controller in self.controllers_list:
            self._methods_dict.update(controller.get_methods_dict())
        pass

    def get_method_by_name(self, method_name):
        return self._methods_dict[method_name]

