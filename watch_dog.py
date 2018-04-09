import rospy


class WatchDog(object):
    def __init__(self, timeOut=0.1, cmdName="", verbose=False):
        self.expire_duration = rospy.Duration.from_sec(timeOut)
        self.next_cmd_expected_time = rospy.Time.now()
        self.initialized = False
        self.cmd_name = cmdName
        self.verbose = verbose

    def acknowledge_wd(self):
        self.initialized = True
        self.next_cmd_expected_time = rospy.Time.now() + self.expire_duration

    def is_wd_expired(self):
        if rospy.Time.now() > self.next_cmd_expected_time and self.initialized:
            if self.verbose is True:
                self.console_print()
            return True
        else:
            return False

    def console_print(self):
        if self.initialized:
            print 'Watch Dog Expired for {} command'.format(self.cmdName)
            self.initialized = False
