from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState


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
        self.topic_type_dict[prefix + 'move_jp'] = [JointState, self._move_jp_cb]
        self.topic_type_dict[prefix + 'move_jr'] = [JointState, self._move_jr_cb]
        self.topic_type_dict[prefix + 'move_cp'] = [JointState, self._move_cp_cb]
        self.topic_type_dict[prefix + 'move_cr'] = [JointState, self._move_cr_cb]

    def _interpolate_cp_cb(self, data):
        pass

    def _interpolate_cr_cb(self, data):
        pass

    def _interpolate_cv_cb(self, data):
        pass

    def _interpolate_cf_cb(self, data):
        pass

    def _interpolate_jp_cb(self, data):
        pass

    def _interpolate_jr_cb(self, data):
        pass

    def _interpolate_jv_cb(self, data):
        pass

    def _interpolate_jf_cb(self, data):
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
