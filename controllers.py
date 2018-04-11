from interpolate_logic import Interpolate
from servo_logic import Servo
from move_logic import Move


class Controllers(object):
    def __init__(self):
        self.controllers_list = [Interpolate(), Servo(), Move()]
        self.methods_dict = dict()
        for controller in self.controllers_list:
            self.methods_dict.update(controller.methods_dict)
        pass

