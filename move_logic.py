import numpy


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
