import rospy


def get_cur_time():
    return rospy.Time.now().to_sec()

