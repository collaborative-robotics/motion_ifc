import rospy
import numpy as np
from topic_type_map import TopicTypeMap


class CommLogic(TopicTypeMap):
    def __init__(self):
        super(CommLogic, self).__init__()
        self._n_handle = rospy.init_node("motion_interface")
        self._sub_list = []
        self._rate = rospy.Rate(10)
        self._counter = 0

        for key, dtpye_cb in self.topic_type_dict.iteritems():
            self._sub_list.append(rospy.Subscriber(key, dtpye_cb[0], dtpye_cb[1], queue_size=10))
        pass

    def run(self):
        while not rospy.is_shutdown():
            self._rate.sleep()
            print 'Running: i = {}'.format(self._counter)
            self._counter = self._counter + 1
            pass
        self._clean()

    def _clean(self):
        for sub in self._sub_list:
            sub.unregister()
