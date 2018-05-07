from motion_cmd_ifc import MotionCmdIfc
import rospy


class MotionIfc(object):
    def __init__(self):
        self._n_handle = rospy.init_node("motion_interface")
        self._rate = rospy.Rate(500)
        self._counter = 0
        self.motion_cmd_ifc = MotionCmdIfc()

    def run(self):
        while not rospy.is_shutdown():
            active_ifcs_list = self.motion_cmd_ifc.get_active_ifcs()
            if active_ifcs_list.__len__() > 0:
                active_ifcs_list[0].execute_controller()
            self._counter = self._counter + 1
            self._rate.sleep()

        self.motion_cmd_ifc.clean()
