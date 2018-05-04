import rospy


class WatchDog(object):
    def __init__(self, time_out=0.1):
        self.expire_duration = rospy.Duration.from_sec(time_out)
        self.next_cmd_expected_time = rospy.Time.now()

    def acknowledge_wd(self):
        self.next_cmd_expected_time = rospy.Time.now() + self.expire_duration

    def is_expired(self):
        if rospy.Time.now() >= self.next_cmd_expected_time:
            return True
        else:
            return False

    def set_timeout(self, time_out):
        self.expire_duration = rospy.Duration.from_sec(time_out)
