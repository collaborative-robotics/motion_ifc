from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from watch_dog import WatchDog


class TopicTypeMap(object):
    def __init__(self):
        prefix = '/motion_ifc/'
        self.topic_type_dict = {}
        self.topic_type_dict[prefix + 'interpolate_cp'] = [TransformStamped, self._interpolate_cp_cb]
        self.topic_type_dict[prefix + 'interpolate_cr'] = [TransformStamped, self._interpolate_cr_cb]
        self.topic_type_dict[prefix + 'interpolate_cv'] = [TransformStamped, self._interpolate_cv_cb]
        self.topic_type_dict[prefix + 'interpolate_cf'] = [TransformStamped, self._interpolate_cf_cb]
        self.topic_type_dict[prefix + 'interpolate_jp'] = [JointState, self._interpolate_jp_cb]
        self.topic_type_dict[prefix + 'interpolate_jr'] = [JointState, self._interpolate_jr_cb]
        self.topic_type_dict[prefix + 'interpolate_jv'] = [JointState, self._interpolate_jv_cb]
        self.topic_type_dict[prefix + 'interpolate_jf'] = [JointState, self._interpolate_jf_cb]
        self.topic_type_dict[prefix + 'servo_cp'] = [TransformStamped, self._servo_cp_cb]
        self.topic_type_dict[prefix + 'servo_cr'] = [TransformStamped, self._servo_cr_cb]
        self.topic_type_dict[prefix + 'servo_cv'] = [TransformStamped, self._servo_cv_cb]
        self.topic_type_dict[prefix + 'servo_cf'] = [TransformStamped, self._servo_cf_cb]
        self.topic_type_dict[prefix + 'servo_jp'] = [JointState, self._servo_jp_cb]
        self.topic_type_dict[prefix + 'servo_jr'] = [JointState, self._servo_jr_cb]
        self.topic_type_dict[prefix + 'servo_jv'] = [JointState, self._servo_jv_cb]
        self.topic_type_dict[prefix + 'servo_jf'] = [JointState, self._servo_jf_cb]
        self.topic_type_dict[prefix + 'move_cp'] = [JointState, self._move_cp_cb]
        self.topic_type_dict[prefix + 'move_cr'] = [JointState, self._move_cr_cb]
        self.topic_type_dict[prefix + 'move_jp'] = [JointState, self._move_jp_cb]
        self.topic_type_dict[prefix + 'move_jr'] = [JointState, self._move_jr_cb]

        self.interpolate_cp_cmd = TransformStamped
        self.interpolate_cr_cmd = TransformStamped
        self.interpolate_cv_cmd = TransformStamped
        self.interpolate_cf_cmd = TransformStamped
        self.interpolate_jp_cmd = JointState
        self.interpolate_jr_cmd = JointState
        self.interpolate_jv_cmd = JointState
        self.interpolate_jf_cmd = JointState
        self.servo_cp_cmd = TransformStamped
        self.servo_cr_cmd = TransformStamped
        self.servo_cv_cmd = TransformStamped
        self.servo_cf_cmd = TransformStamped
        self.servo_jp_cmd = JointState
        self.servo_jr_cmd = JointState
        self.servo_jv_cmd = JointState
        self.servo_jf_cmd = JointState
        self.move_cp_cmd = JointState
        self.move_cr_cmd = JointState
        self.move_jp_cmd = JointState
        self.move_jr_cmd = JointState

        self.cmd_list = [
            self.interpolate_cp_cmd,
            self.interpolate_cr_cmd,
            self.interpolate_cv_cmd,
            self.interpolate_cf_cmd,
            self.interpolate_jp_cmd,
            self.interpolate_jr_cmd,
            self.interpolate_jv_cmd,
            self.interpolate_jf_cmd,
            self.servo_cp_cmd,
            self.servo_cr_cmd,
            self.servo_cv_cmd,
            self.servo_cf_cmd,
            self.servo_jp_cmd,
            self.servo_jr_cmd,
            self.servo_jv_cmd,
            self.servo_jf_cmd,
            self.move_cp_cmd,
            self.move_cr_cmd,
            self.move_jp_cmd,
            self.move_jr_cmd
        ]

        self.interpolate_cp_wd = WatchDog()
        self.interpolate_cr_wd = WatchDog()
        self.interpolate_cv_wd = WatchDog()
        self.interpolate_cf_wd = WatchDog()
        self.interpolate_jp_wd = WatchDog()
        self.interpolate_jr_wd = WatchDog()
        self.interpolate_jv_wd = WatchDog()
        self.interpolate_jf_wd = WatchDog()

        self.wd_list = [
            self.interpolate_cp_wd,
            self.interpolate_cr_wd,
            self.interpolate_cv_wd,
            self.interpolate_cf_wd,
            self.interpolate_jp_wd,
            self.interpolate_jr_wd,
            self.interpolate_jv_wd,
            self.interpolate_jf_wd
        ]

    def _interpolate_cp_cb(self, data):
        self.interpolate_cp_cmd = data
        self.interpolate_cp_wd.acknowledge_wd()
        pass

    def _interpolate_cr_cb(self, data):
        self.interpolate_cr_cmd = data
        self.interpolate_cr_wd.acknowledge_wd()
        pass

    def _interpolate_cv_cb(self, data):
        self.interpolate_cv_cmd = data
        self.interpolate_cv_wd.acknowledge_wd()
        pass

    def _interpolate_cf_cb(self, data):
        self.interpolate_cf_cmd = data
        self.interpolate_cf_wd.acknowledge_wd()
        pass

    def _interpolate_jp_cb(self, data):
        self.interpolate_jp_cmd = data
        self.interpolate_jp_wd.acknowledge_wd()
        pass

    def _interpolate_jr_cb(self, data):
        self.interpolate_jr_cmd = data
        self.interpolate_jr_wd.acknowledge_wd()
        pass

    def _interpolate_jv_cb(self, data):
        self.interpolate_jv_cmd = data
        self.interpolate_jv_wd.acknowledge_wd()
        pass

    def _interpolate_jf_cb(self, data):
        self.interpolate_jf_cmd = data
        self.interpolate_jf_wd.acknowledge_wd()
        pass

    def _servo_cp_cb(self, data):
        pass

    def _servo_cr_cb(self, data):
        pass

    def _servo_cv_cb(self, data):
        pass

    def _servo_cf_cb(self, data):
        pass

    def _servo_jp_cb(self, data):
        pass

    def _servo_jr_cb(self, data):
        pass

    def _servo_jv_cb(self, data):
        pass

    def _servo_jf_cb(self, data):
        pass

    def _move_cp_cb(self, data):
        pass

    def _move_cr_cb(self, data):
        pass

    def _move_jp_cb(self, data):
        pass

    def _move_jr_cb(self, data):
        pass
