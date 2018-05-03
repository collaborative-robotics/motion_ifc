import rospy
from watch_dog import WatchDog
from crkt_common import *


class CommunicationIfc(object):
    def __init__(self, topic_name, data_type, enable_wd=False, queue_size=10, is_publisher=False):
        self.topic_name = topic_name
        self._data = data_type()
        self.enable_wd = enable_wd
        self.crtk_name = ''
        self.wd_time_out = 0.0
        self.last_received_time = 0.0

        self._is_publisher = is_publisher
        self.watch_dog = WatchDog(self.wd_time_out)
        if is_publisher:
            self.pub = rospy.Publisher(topic_name, data_type, queue_size=queue_size)
        else:
            self.sub = rospy.Subscriber(topic_name, data_type, self.message_cb, queue_size=queue_size)

    def message_cb(self, data):
        self._data = data
        self.last_received_time = rospy.Time.now().to_sec()
        if self.enable_wd:
            self.watch_dog.acknowledge_wd()

    def is_active(self):
        return not self.watch_dog.is_wd_expired()

    def get_data(self):
        if self._is_publisher:
            print 'WARN: This is a publishing interface'
        return self._data

    def set_data(self, data):
        if not self._is_publisher:
            print 'WARN: This is an output interface, you cannot write data to it'
        else:
            if not isinstance(data, self._data):
                print 'WARN: Mismatched data-types, ignoring'
            else:
                self._data = data

    def get_last_received_time(self):
        return self.last_received_time

    def get_crtk_name(self):
        self.crtk_name = interpret_crtk_name_from_topic(self.topic_name)
        return self.crtk_name

    def get_mode_str(self):
        crtk_name = self.get_crtk_name()
        return crtk_name.split('_')[0]

    def get_opspace_char(self):
        crtk_name = self.get_crtk_name()
        return crtk_name.split('_')[1][0]

    def get_controller_char(self):
        crtk_name = self.get_crtk_name()
        return crtk_name.split('_')[1][1]
