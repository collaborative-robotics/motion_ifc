from motion_cmd_ifc import MotionCmdIfc
import rospy


class MotionIfc(object):
    def __init__(self):
        self._n_handle = rospy.init_node("motion_interface")
        self._rate = rospy.Rate(1)
        self._counter = 0
        self.motion_cmd_ifc = MotionCmdIfc()

    def run(self):
        while not rospy.is_shutdown():
            active_ifcs_list = self.motion_cmd_ifc.get_active_ifcs()
            last_active_ifc = self.motion_cmd_ifc.get_last_activated_ifc()
            if not last_active_ifc is None:
                print 'Lastest active ifc: {}'.format(last_active_ifc.get_crtk_name())
            if active_ifcs_list.__len__() > 0:
                ifc = active_ifcs_list[0]
                ifc.motion_controller_write(ifc.get_data())
            self._counter = self._counter + 1
            self._rate.sleep()

        self.motion_cmd_ifc.clean()
