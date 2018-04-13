import rospy
from watch_dog import WatchDog
from crkt_common import *


class CommunicationIfc(object):
    def __init__(self, topic_name, data_type, enable_wd=False, queue_size=10):
        self.topic_name = topic_name
        self.cmd = data_type()
        self.enable_wd = enable_wd
        self.crtk_name = interpret_crtk_name_from_topic(topic_name)
        self.wd_time_out = 0.0
        self.last_received_time = 0.0

        self.watch_dog = WatchDog(self.wd_time_out)
        self.sub = rospy.Subscriber(topic_name, data_type, self.message_cb, queue_size=queue_size)

    def message_cb(self, data):
        self.cmd = data
        self.last_received_time = rospy.Time.now().to_sec()
        if self.enable_wd:
            self.watch_dog.acknowledge_wd()

    def is_active(self):
        return not self.watch_dog.is_wd_expired()

    def get_last_received_time(self):
        return self.last_received_time

    def get_crtk_name(self):
        return self.crtk_name