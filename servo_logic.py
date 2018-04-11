import numpy


class Servo(object):
    def __init__(self):
        self.methods_dict = dict()
        self.methods_dict['servo_cp'] = self.servo_cp
        self.methods_dict['servo_cr'] = self.servo_cr
        self.methods_dict['servo_cv'] = self.servo_cv
        self.methods_dict['servo_cf'] = self.servo_cf
        self.methods_dict['servo_jp'] = self.servo_jp
        self.methods_dict['servo_jr'] = self.servo_jr
        self.methods_dict['servo_jv'] = self.servo_jv
        self.methods_dict['servo_jf'] = self.servo_jf

    def servo_cp(self):
        pass

    def servo_cr(self):
        pass

    def servo_cv(self):
        pass

    def servo_cf(self):
        pass

    def servo_jp(self):
        pass

    def servo_jr(self):
        pass

    def servo_jv(self):
        pass

    def servo_jf(self):
        pass
