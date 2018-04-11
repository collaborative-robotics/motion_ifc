import numpy


class Interpolate(object):
    def __init__(self):
        self.methods_dict = dict()
        self.methods_dict['interpolate_cp'] = self.interpolate_cp
        self.methods_dict['interpolate_cr'] = self.interpolate_cr
        self.methods_dict['interpolate_cv'] = self.interpolate_cv
        self.methods_dict['interpolate_cf'] = self.interpolate_cf
        self.methods_dict['interpolate_jp'] = self.interpolate_jp
        self.methods_dict['interpolate_jr'] = self.interpolate_jr
        self.methods_dict['interpolate_jv'] = self.interpolate_jv
        self.methods_dict['interpolate_jf'] = self.interpolate_jf

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
