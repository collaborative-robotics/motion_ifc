import rospy
from watch_dog import WatchDog
from crkt_common import *


class CommunicationIfc(object):
    def __init__(self, topic_name, data_type, enable_wd=False, queue_size=10, is_publisher=False):
        self._topic_name = topic_name
        self._data = data_type()
        self._enable_wd = enable_wd
        self._crtk_name = ''
        self._last_received_time = 0.0
        self._is_publisher = is_publisher
        self._is_data_new = False
        self._watch_dog = WatchDog()
        if is_publisher:
            self._pub = rospy.Publisher(topic_name, data_type, queue_size=queue_size)
        else:
            self._sub = rospy.Subscriber(topic_name, data_type, self.message_cb, queue_size=queue_size)

    def is_data_new(self):
        return self._is_data_new

    def message_cb(self, data):
        self._data = data
        self._last_received_time = rospy.Time.now().to_sec()
        if self._enable_wd:
            self._watch_dog.acknowledge_wd()
        self._is_data_new = True

    def is_active(self):
        return not self._watch_dog.is_expired()

    def get_data(self):
        if self._is_publisher:
            print 'WARN: This is a publishing interface'
        self._is_data_new = False
        return self._data

    def set_data(self, data):
        if not self._is_publisher:
            print 'WARN: This is an output interface, you cannot write data to it'
        else:
            if not isinstance(data, type(self._data)):
                print 'WARN: Mismatched data-types, ignoring'
                print 'Given type {}'.format(type(data))
                print 'Expected type {}'.format(type(self._data))
            else:
                self._data = data
                self._pub.publish(data)

    def get_last_received_time(self):
        return self._last_received_time

    def get_crtk_name(self):
        self._crtk_name = interpret_crtk_name_from_topic(self._topic_name)
        return self._crtk_name

    def get_mode_as_str(self):
        crtk_name = self.get_crtk_name()
        return crtk_name.split('_')[0]

    def get_opspace_as_str(self):
        crtk_name = self.get_crtk_name()
        return crtk_name.split('_')[1][0]

    def get_controller_as_str(self):
        crtk_name = self.get_crtk_name()
        return crtk_name.split('_')[1][1]

    def disconnect(self):
        self._sub.unregister()
