import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt


class Interpolation(object):
    def __init__(self):
        self._t0 = 0.0
        self._tf = 0.0

        self._coefficients = []
        self._T_mat = []
        self._boundary_conditions = []

        self._dimensions = 1
        self._t_array = []
        pass

    def _compute_time_mat(self, t0, tf):
        T_mat = np.mat([[1, t0, t0**2,     t0**3,      t0**4,      t0**5],
                        [0,  1,  2*t0, 3*(t0**2),  4*(t0**3),  5*(t0**4)],
                        [0,  0,     2,      6*t0, 12*(t0**2), 20*(t0**3)],
                        [1, tf, tf**2,     tf**3,      tf**4,      tf**5],
                        [0,  1,  2*tf, 3*(tf**2),  4*(tf**3),  5*(tf**4)],
                        [0,  0,     2,      6*tf, 12*(tf**2), 20*(tf**3)]])
        return T_mat

    def pva_interpolated(self, t, coefficients):
        pos = self.p_interpolated(t, coefficients)
        vel = self.v_interpolated(t, coefficients)
        acc = self.a_interpolated(t, coefficients)
        return [pos, vel, acc]

    def p_interpolated(self, t, coefficients):
            if t.__len__() == 1:
                if t < self._t0 or t > self._tf:
                    raise Exception('Time should be between t0 and tf')
            c = coefficients
            pos = c[0] + c[1] * t + c[2] * (t**2) + c[3] * (t**3) + c[4] * (t**4) + c[5] * (t**5)
            return pos

    def v_interpolated(self, t, coefficients):
            if t.__len__() == 1:
                if t < self._t0 or t > self._tf:
                    raise Exception('Time should be between t0 and tf')
            c = coefficients
            vel = c[1] + 2 * c[2] * t + 3 * c[3] * (t**2) + 4 * c[4] * (t**3) + 5 * c[5] * (t**4)
            return vel

    def a_interpolated(self, t, coeff):
            if t.__len__() == 1:
                if t < self._t0 or t > self._tf:
                    raise Exception('Time should be between t0 and tf')
            acc = 2 * coeff[2] + 6 * coeff[3] * t + 12 * coeff[4] * (t**2) + 20 * coeff[5] * (t**3)
            return acc

    def compute_interpolation_params(self, p0, pf, v0, vf, a0, af, t0, tf):
        if p0.__len__() != pf.__len__() or v0.__len__() != vf.__len__() or a0.__len__() != af.__len__():
            raise Exception('All arrays for initial and final P,V & A must be of same length')

        self._dimensions = p0.__len__()
        dims = self._dimensions
        print 'Number of Dimensions = {}'.format(dims)

        self._boundary_conditions = np.zeros([dims, 6])
        self._coefficients = np.zeros([6, dims])
        self._T_mat = np.zeros([6, 6, dims])

        self._t0 = t0
        self._tf = tf

        for i in range(0, dims):
            self._boundary_conditions[i, :] = np.mat([p0[i], v0[i], a0[i], pf[i], vf[i], af[i]])
            self._T_mat[:, :, i] = self._compute_time_mat(t0, tf)
            self._coefficients[:, i] = np.matmul(np.linalg.inv(self._T_mat[:, :, i]),
                                                 np.transpose(self._boundary_conditions[i, :]))

    def plot_trajectory(self, n_steps=50):
        if n_steps < 5:
            raise Exception('n_steps is very low, provide a value greater than 5')
        n = n_steps
        self._t_array = np.linspace(self._t0, self._tf, n)

        p = np.zeros([n, self._dimensions])
        v = np.zeros([n, self._dimensions])
        a = np.zeros([n, self._dimensions])

        for i in range(0, self._dimensions):
            [p[:, i], v[:, i], a[:, i]] = self.pva_interpolated(self._t_array, self._coefficients[:, i])

        plt.plot(self._t_array, p, 'o',
                 self._t_array, v, '-',
                 self._t_array, a, '--')
        plt.legend([['position'], 'velocity', 'acceleration'], loc='best')
        plt.show()


obj = Interpolation()
obj.compute_interpolation_params([ 0.5, 1.0,-0.3],
                                 [-0.5, 0.0, 0.2],
                                 [ 0.0, 0.0,-0.2],
                                 [ 0.5, 0.0, 0.0],
                                 [ 0.0, 0.0, 0.0],
                                 [-0.3, 0.0, 0.0],
                                   0.0, 10.0)
obj.plot_trajectory(500)