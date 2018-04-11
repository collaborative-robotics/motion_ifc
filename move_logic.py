import numpy


# Each sub-controller must provide a dict() of all the methods with the method
# name as the key and the method handle as the value so that the Controller
# class can append all the dict() items in a unified way
class Move(object):
    def __init__(self):
        self.methods_dict = dict()
        self.methods_dict['move_cp'] = self.move_cp
        self.methods_dict['move_cr'] = self.move_cr
        self.methods_dict['move_jp'] = self.move_jp
        self.methods_dict['move_jr'] = self.move_jr

    def move_cp(self):
        pass

    def move_cr(self):
        pass

    def move_jp(self):
        pass

    def move_jr(self):
        pass
