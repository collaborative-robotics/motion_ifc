from motion_cmd_ifc import MotionCmdIfc
from robot_state_ifc import RobotStateIfc
import rospy


class MotionIfc(object):
    def __init__(self):
        self._n_handle = rospy.init_node("motion_interface")
        self._rate = rospy.Rate(1)
        self._counter = 0
        self.robot_cmd_ifc = MotionCmdIfc()
        self.robot_state_ifc = RobotStateIfc()

    def run(self):
        while not rospy.is_shutdown():
            active_ifcs_list = self.robot_cmd_ifc.get_active_ifcs()
            last_active_ifc = self.robot_cmd_ifc.get_last_activated_ifc()
            #print 'i: {}, Number of active interfaces: {}'.format(self._counter, active_ifcs_list.__len__())
            if not last_active_ifc is None:
                print 'Lastest active ifc: {}'.format(last_active_ifc.get_crtk_name())
            if active_ifcs_list.__len__() > 0:
                ifc = active_ifcs_list[0]
                ifc.control_method(ifc.get_data(), ifc.state_method())
            self._counter = self._counter + 1
            self._rate.sleep()

        self.robot_cmd_ifc.clean()
        self.robot_state_ifc.clean()
