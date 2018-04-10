import rospy
import numpy as np
from watch_dog import WatchDog
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState


class CommIfc(object):
    def __init__(self, topic_name, data_type, enable_wd = False, queue_size=10):
        self.topic_name = topic_name
        self.cmd = data_type()
        self.watch_dog = WatchDog
        self.enable_wd = enable_wd

        self.sub = rospy.Subscriber(topic_name, data_type, self.message_cb, queue_size=queue_size)

    def message_cb(self, data):
        self.cmd = data
        if self.enable_wd:
            self.watch_dog.acknowledge_wd()


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
