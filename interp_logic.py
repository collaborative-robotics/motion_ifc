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

    def get_interpolated_x_dx_ddx(self, t):
        return self.get_interpolated_x(t), self.get_interpolated_dx(t), self.get_interpolated_ddx(t)

    def get_interpolated_x(self, t):
            if not type(t) is np.array:
                t = np.array(t)
            # t = np.array(t)
            if t.size == 1:
                if not self._t0 <= t <= self._tf:
                    raise Exception('Time {} should be between {} and {}'.format(t, self._t0, self._tf))
            pos = np.zeros([t.size, self._dimensions])
            for i in range(0, self._dimensions):
                c = self._coefficients[:, i]
                pos[:, i] = c[0] + c[1] * t + c[2] * (t**2) + c[3] * (t**3) + c[4] * (t**4) + c[5] * (t**5)
            return pos

    def get_interpolated_dx(self, t):
            if not type(t) is np.array:
                t = np.array(t)
            # t = np.array(t)
            if t.size == 1:
                if t < self._t0 or t > self._tf:
                    raise Exception('Time should be between {} and {}'.format(self._t0, self._tf))
            vel = np.zeros([t.size, self._dimensions])
            for i in range(0, self._dimensions):
                c = self._coefficients[:, i]
                vel[:, i] = c[1] + 2 * c[2] * t + 3 * c[3] * (t**2) + 4 * c[4] * (t**3) + 5 * c[5] * (t**4)
            return vel

    def get_interpolated_ddx(self, t):
            if not type(t) is np.array:
                t = np.array(t)
            # t = np.array(t)
            if t.size == 1:
                if t < self._t0 or t > self._tf:
                    raise Exception('Time should be between t0 and tf')
            acc = np.zeros([t.size, self._dimensions])
            for i in range(0, self._dimensions):
                c = self._coefficients[:, i]
                acc[:, i] = 2 * c[2] + 6 * c[3] * t + 12 * c[4] * (t**2) + 20 * c[5] * (t**3)
            return acc

    def get_t0(self):
        return self._t0

    def get_tf(self):
        return self._tf

    def compute_interpolation_params(self, x0, xf, dx0, dxf, ddx0, ddxf, t0, tf):
        if not type(x0) is np.array:
            x0 = np.asarray(x0)
        if not type(xf) is np.array:
            xf = np.asarray(xf)
        if not type(dx0) is np.array:
            dx0 = np.asarray(dx0)
        if not type(dxf) is np.array:
            dxf = np.asarray(dxf)
        if not type(ddx0) is np.array:
            ddx0 = np.asarray(ddx0)
        if not type(ddxf) is np.array:
            ddxf = np.asarray(ddxf)
        if x0.size != xf.size or dx0.size != dxf.size or ddx0.size != ddxf.size:
            raise Exception('All arrays for initial and final P,V & A must be of same length')

        self._dimensions = x0.size
        dims = self._dimensions

        self._boundary_conditions = np.zeros([dims, 6])
        self._coefficients = np.zeros([6, dims])
        self._T_mat = np.zeros([6, 6, dims])

        self._t0 = t0
        self._tf = tf

        for i in range(0, dims):
            self._boundary_conditions[i, :] = np.mat([x0[i], dx0[i], ddx0[i], xf[i], dxf[i], ddxf[i]])
            self._T_mat[:, :, i] = self._compute_time_mat(t0, tf)
            self._coefficients[:, i] = np.matmul(np.linalg.inv(self._T_mat[:, :, i]),
                                                 np.transpose(self._boundary_conditions[i, :]))

    def plot_trajectory(self, n_steps=50, t0=None, tf=None):
        if n_steps < 5:
            raise Exception('n_steps is very low, provide a value greater than 5')
        n = n_steps
        if t0 is None:
            t0 = self._t0
        if tf is None:
            tf = self._tf
        self._t_array = np.linspace(t0, tf, n)

        p, v, a = self.get_interpolated_x_dx_ddx(self._t_array)

        plt.plot(self._t_array, p, '-o',
                 self._t_array, v, '-',
                 self._t_array, a, '--')
        p_legend_str = [''] * self._dimensions
        v_legend_str = [''] * self._dimensions
        a_legend_str = [''] * self._dimensions
        for i in range(0, self._dimensions):
            p_legend_str[i] = 'pos '
            v_legend_str[i] = 'vel '
            a_legend_str[i] = 'acc '
        plt.legend(p_legend_str, loc='best')
        plt.show()
