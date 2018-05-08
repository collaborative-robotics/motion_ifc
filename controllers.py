from interp_logic import Interpolation
import rospy
from threading import Thread
from crkt_common import *
from robot_cmd_ifc import RobotCmdIfc
from robot_state_ifc import RobotStateIfc
import tf_conversions


# Each sub-controller must provide a dict() of all the methods with the method
# name as the key and the method handle as the value so that this Controller
# class and append all the dict() items in a unified way


class ControllerData:
    def __init__(self, controller_name, robot_cmd_ifc, robot_state_ifc):
        self.interpolater = Interpolation()
        self._active = False
        self._controller_name = controller_name
        # New we are connecting the state methods using the naming from the crtk_name of cmd
        # There is no crtk_name for relative 'r' feedback as of yet, so if the command is relative, always bind the
        # position 'p' state method
        # Additionally, there is not 'p', 'v', 'f' or 'r' for joint space feedback, we only have joint states,
        # so if that's the command always bind to 's' for op_space
        op_space_str = get_opspace_as_str(controller_name)
        controller_str = get_controller_as_str(controller_name)
        controller_feedback_str = controller_str

        if op_space_str is 'j':
            controller_feedback_str = 's'
            if controller_str is 'p':
                self._conversion_method = np_array_to_joint_state_pos
            elif controller_str is 'v':
                self._conversion_method = np_array_to_joint_state_vel
            elif controller_str is 'f':
                self._conversion_method = np_array_to_joint_state_effort
            elif controller_str is 'r':
                self._conversion_method = np_array_to_joint_state_pos
                controller_str = 'p'
            else:
                raise Exception('DONTBEANONION')
        elif op_space_str is 'c':
            self._conversion_method = np_array_to_transform_stamped
            if controller_str is 'r':
                controller_str = 'p'
                controller_feedback_str = 'p'

        state_method_name = 'measured_' + op_space_str + controller_feedback_str
        command_method_name = 'servo_' + op_space_str + controller_str
        self.get_robot_state = robot_state_ifc.get_method_by_name(state_method_name)
        self.set_robot_command = robot_cmd_ifc.get_method_by_name(command_method_name)
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
        return self._conversion_method(xt)

    def calculate_dt(self, cur_time):
        dt = cur_time - self._last_call_time
        self._last_call_time = cur_time
        if 0 < dt <= self._dt_max:
            self._dt = dt
        return self._dt


class Interpolate(object):
    def __init__(self, robot_cmd_ifc, robot_state_ifc, t_start):
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

        self._cp_ctrl = ControllerData('interpolate_cp', robot_cmd_ifc, robot_state_ifc)
        self._cr_ctrl = ControllerData('interpolate_cr', robot_cmd_ifc, robot_state_ifc)
        self._cv_ctrl = ControllerData('interpolate_cv', robot_cmd_ifc, robot_state_ifc)
        self._cf_ctrl = ControllerData('interpolate_cf', robot_cmd_ifc, robot_state_ifc)
        self._jp_ctrl = ControllerData('interpolate_jp', robot_cmd_ifc, robot_state_ifc)
        self._jr_ctrl = ControllerData('interpolate_jr', robot_cmd_ifc, robot_state_ifc)
        self._jv_ctrl = ControllerData('interpolate_jv', robot_cmd_ifc, robot_state_ifc)
        self._jf_ctrl = ControllerData('interpolate_jf', robot_cmd_ifc, robot_state_ifc)

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

    def interpolate_cr(self, cmd):
        state = self._cr_ctrl.get_robot_state()
        if cmd is not None and state is not None:
            # t_robot = transform_stamped_to_frame(state)
            # t_relative = transform_stamped_to_frame(cmd)
            cmd = transform_stamped_to_frame(state) * transform_stamped_to_frame(cmd)
            p0 = transform_stamped_to_np_array(state)
            pf = frame_to_np_array(cmd)
            v0 = [0, 0, 0, 0, 0, 0]
            vf = [0, 0, 0, 0, 0, 0]
            a0 = [0, 0, 0, 0, 0, 0]
            af = [0, 0, 0, 0, 0, 0]

            cur_time = rospy.Time.now().to_sec()
            t0 = cur_time - self._t_start
            tf = t0 + self._cr_ctrl.calculate_dt(t0)
            self._cr_ctrl.interpolater.compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)
            self._cr_ctrl.set_active()
        pass

    def interpolate_cv(self, cmd):
        pass

    def interpolate_cf(self, cmd):
        state = self._cf_ctrl.get_robot_state()
        if cmd is not None and state is not None:
            f0 = transform_stamped_to_np_array(state)
            ff = transform_stamped_to_np_array(cmd)
            df0 = [0, 0, 0, 0, 0, 0]
            dff = [0, 0, 0, 0, 0, 0]
            ddf0 = [0, 0, 0, 0, 0, 0]
            ddff = [0, 0, 0, 0, 0, 0]

            cur_time = rospy.Time.now().to_sec()
            t0 = cur_time - self._t_start
            tf = t0 + self._cf_ctrl.calculate_dt(t0)
            self._cf_ctrl.interpolater.compute_interpolation_params(f0, ff, df0, dff, ddf0, ddff, t0, tf)
            self._cf_ctrl.set_active()
        pass

    def interpolate_jp(self, cmd):
        state = self._jp_ctrl.get_robot_state()
        if cmd is not None and state is not None:
            j0 = state.position
            jf = cmd.position
            v0 = [0, 0, 0, 0, 0, 0]
            vf = [0, 0, 0, 0, 0, 0]
            a0 = [0, 0, 0, 0, 0, 0]
            af = [0, 0, 0, 0, 0, 0]

            cur_time = rospy.Time.now().to_sec()
            t0 = cur_time - self._t_start
            tf = t0 + self._jp_ctrl.calculate_dt(t0)
            self._jp_ctrl.interpolater.compute_interpolation_params(j0, jf, v0, vf, a0, af, t0, tf)
            self._jp_ctrl.set_active()
        pass

    def interpolate_jr(self, cmd):
        state = self._jr_ctrl.get_robot_state()
        if cmd is not None and state is not None:
            j0 = np.array(state.position)
            jf = j0 + np.array(cmd.position)
            v0 = [0, 0, 0, 0, 0, 0]
            vf = [0, 0, 0, 0, 0, 0]
            a0 = [0, 0, 0, 0, 0, 0]
            af = [0, 0, 0, 0, 0, 0]

            cur_time = rospy.Time.now().to_sec()
            t0 = cur_time - self._t_start
            tf = t0 + self._jr_ctrl.calculate_dt(t0)
            self._jr_ctrl.interpolater.compute_interpolation_params(j0, jf, v0, vf, a0, af, t0, tf)
            self._jr_ctrl.set_active()
        pass

    def interpolate_jv(self, cmd):
        pass

    def interpolate_jf(self, cmd):
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
    def __init__(self, robot_cmd_ifc, robot_state_ifc, t_start):
        self._t_start = t_start
        self._methods_dict = dict()
        self._methods_dict['move_cp'] = self.move_cp
        self._methods_dict['move_cr'] = self.move_cr
        self._methods_dict['move_jp'] = self.move_jp
        self._methods_dict['move_jr'] = self.move_jr

        self._cp_ctrl = ControllerData('move_cp', robot_cmd_ifc, robot_state_ifc)
        self._cr_ctrl = ControllerData('move_cr', robot_cmd_ifc, robot_state_ifc)
        self._jp_ctrl = ControllerData('move_jp', robot_cmd_ifc, robot_state_ifc)
        self._jr_ctrl = ControllerData('move_jr', robot_cmd_ifc, robot_state_ifc)

        self._controller_data_list = [self._cp_ctrl,
                                      self._cr_ctrl,
                                      self._jp_ctrl,
                                      self._jr_ctrl]

        self._thread = Thread(target=self._execute)
        self._thread.start()

    def move_cp(self, cmd):
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
            while self._cp_ctrl.is_active():
                a = 0 # Wait/Block
        pass

    def move_cr(self, cmd):
        state = self._cr_ctrl.get_robot_state()
        if cmd is not None and state is not None:
            cmd = transform_stamped_to_frame(state) * transform_stamped_to_frame(cmd)
            p0 = transform_stamped_to_np_array(state)
            pf = frame_to_np_array(cmd)
            v0 = [0, 0, 0, 0, 0, 0]
            vf = [0, 0, 0, 0, 0, 0]
            a0 = [0, 0, 0, 0, 0, 0]
            af = [0, 0, 0, 0, 0, 0]

            cur_time = rospy.Time.now().to_sec()
            t0 = cur_time - self._t_start
            tf = t0 + self._cr_ctrl.calculate_dt(t0)
            self._cr_ctrl.interpolater.compute_interpolation_params(p0, pf, v0, vf, a0, af, t0, tf)
            self._cr_ctrl.set_active()
            while self._cr_ctrl.is_active():
                a = 0 # Wait/Block
        pass

    def move_jp(self, cmd):
        state = self._jp_ctrl.get_robot_state()
        if cmd is not None and state is not None:
            j0 = state.position
            jf = cmd.position
            v0 = [0, 0, 0, 0, 0, 0]
            vf = [0, 0, 0, 0, 0, 0]
            a0 = [0, 0, 0, 0, 0, 0]
            af = [0, 0, 0, 0, 0, 0]

            cur_time = rospy.Time.now().to_sec()
            t0 = cur_time - self._t_start
            tf = t0 + self._jp_ctrl.calculate_dt(t0)
            self._jp_ctrl.interpolater.compute_interpolation_params(j0, jf, v0, vf, a0, af, t0, tf)
            self._jp_ctrl.set_active()
            while self._jp_ctrl.is_active():
                a = 0 # Wait/Block
        pass

    def move_jr(self, cmd):
        state = self._jr_ctrl.get_robot_state()
        if cmd is not None and state is not None:
            j0 = np.array(state.position)
            jf = j0 + np.array(cmd.position)
            v0 = [0, 0, 0, 0, 0, 0]
            vf = [0, 0, 0, 0, 0, 0]
            a0 = [0, 0, 0, 0, 0, 0]
            af = [0, 0, 0, 0, 0, 0]

            cur_time = rospy.Time.now().to_sec()
            t0 = cur_time - self._t_start
            tf = t0 + self._jr_ctrl.calculate_dt(t0)
            self._jr_ctrl.interpolater.compute_interpolation_params(j0, jf, v0, vf, a0, af, t0, tf)
            self._jr_ctrl.set_active()
            while self._jr_ctrl.is_active():
                a = 0 # Wait/Block
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


class Controllers(object):
    def __init__(self, namespace, arm_name, t_start=0.0):
        self._robot_cmd_ifc = RobotCmdIfc(namespace, arm_name)
        self._robot_state_ifc = RobotStateIfc(namespace, arm_name)
        self.controllers_list = [Interpolate(self._robot_cmd_ifc, self._robot_state_ifc, t_start),
                                 Servo(self._robot_cmd_ifc, t_start),
                                 Move(self._robot_cmd_ifc, self._robot_state_ifc, t_start)]
        self._methods_dict = dict()
        for controller in self.controllers_list:
            self._methods_dict.update(controller.get_methods_dict())
        pass

    def get_method_by_name(self, method_name):
        return self._methods_dict[method_name]

    def disconnect(self):
        self._robot_state_ifc.clean()
        self._robot_cmd_ifc.clean()


