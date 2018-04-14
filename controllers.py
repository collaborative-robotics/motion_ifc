# Each sub-controller must provide a dict() of all the methods with the method
# name as the key and the method handle as the value so that this Controller
# class and append all the dict() items in a unified way


class Interpolate(object):
    def __init__(self):
        self._methods_dict = dict()
        self._methods_dict['interpolate_cp'] = self.interpolate_cp
        self._methods_dict['interpolate_cr'] = self.interpolate_cr
        self._methods_dict['interpolate_cv'] = self.interpolate_cv
        self._methods_dict['interpolate_cf'] = self.interpolate_cf
        self._methods_dict['interpolate_jp'] = self.interpolate_jp
        self._methods_dict['interpolate_jr'] = self.interpolate_jr
        self._methods_dict['interpolate_jv'] = self.interpolate_jv
        self._methods_dict['interpolate_jf'] = self.interpolate_jf

    def interpolate_cp(self):
        print 'IMAPOTATO'
        pass

    def interpolate_cr(self):
        pass

    def interpolate_cv(self):
        pass

    def interpolate_cf(self):
        pass

    def interpolate_jp(self):
        pass

    def interpolate_jr(self):
        pass

    def interpolate_jv(self):
        pass

    def interpolate_jf(self):
        pass

    def get_methods_dict(self):
        return self._methods_dict


class Servo(object):
    def __init__(self):
        self._methods_dict = dict()
        self._methods_dict['servo_cp'] = self.servo_cp
        self._methods_dict['servo_cr'] = self.servo_cr
        self._methods_dict['servo_cv'] = self.servo_cv
        self._methods_dict['servo_cf'] = self.servo_cf
        self._methods_dict['servo_jp'] = self.servo_jp
        self._methods_dict['servo_jr'] = self.servo_jr
        self._methods_dict['servo_jv'] = self.servo_jv
        self._methods_dict['servo_jf'] = self.servo_jf

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

    def get_methods_dict(self):
        return self._methods_dict


class Move(object):
    def __init__(self):
        self._methods_dict = dict()
        self._methods_dict['move_cp'] = self.move_cp
        self._methods_dict['move_cr'] = self.move_cr
        self._methods_dict['move_jp'] = self.move_jp
        self._methods_dict['move_jr'] = self.move_jr

    def move_cp(self):
        pass

    def move_cr(self):
        pass

    def move_jp(self):
        pass

    def move_jr(self):
        pass

    def get_methods_dict(self):
        return self._methods_dict


class Controllers(object):
    def __init__(self):
        self.controllers_list = [Interpolate(), Servo(), Move()]
        self._methods_dict = dict()
        for controller in self.controllers_list:
            self._methods_dict.update(controller.get_methods_dict())
        pass

    def get_method_by_name(self, method_name):
        return self._methods_dict[method_name]

