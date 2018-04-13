from interpolate_logic import Interpolate
from servo_logic import Servo
from move_logic import Move


# Each sub-controller must provide a dict() of all the methods with the method
# name as the key and the method handle as the value so that this Controller
# class and append all the dict() items in a unified way
class Controllers(object):
    def __init__(self):
        self.controllers_list = [Interpolate(), Servo(), Move()]
        self.methods_dict = dict()
        for controller in self.controllers_list:
            self.methods_dict.update(controller.methods_dict)
        pass

    def get_method_by_name(self, method_name):
        return self.methods_dict[method_name]

