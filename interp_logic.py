import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt
from threading import Lock


class Interpolation(object):
    def __init__(self):
        self._t0 = 0.0
        self._tf = 0.0

        self._coefficients = np.zeros([1, 1])
        self._T_mat = np.zeros([1, 1, 1])
        self._boundary_conditions = np.zeros([1, 1])

        self._dimensions = 1
        self._t_array = []
        self._x = np.zeros([1, 1])
        self._dx = np.zeros([1, 1])
        self._ddx = np.zeros([1, 1])
        self._lock = Lock()
        # Define a custom margin of error for t0
        self._t0_moe = 0.01
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
        self._lock.acquire()
        if not type(t) is np.array:
            t = np.array(t)
        # t = np.array(t)
        if t.size == 1:
            if not self._t0 - self._t0_moe <= t <= self._tf:
                raise Exception('Time {} should be between {} and {}'.format(t, self._t0, self._tf))
            elif self._t0 - self._t0_moe <= t < self._t0:
                print 'Warning, t: {} < t0: {} but within Margin of Error: {}'.format(t, self._t0, self._t0_moe)
                t = np.array(self._t0)

        if not self._x.shape[0] == t.size:
            self._x = np.zeros([t.size, self._dimensions])

        # Since all interpolation is done before 0.0 and tf, we adjust t accordingly by subtracting self._t0
        t = t - self._t0
        for i in range(0, self._dimensions):
            c = self._coefficients[:, i]
            self._x[:, i] = c[0] + c[1] * t + c[2] * (t**2) + c[3] * (t**3) + c[4] * (t**4) + c[5] * (t**5)
        self._lock.release()
        return self._x

    def get_interpolated_dx(self, t):
        self._lock.acquire()
        if not type(t) is np.array:
            t = np.array(t)
        # t = np.array(t)
        if t.size == 1:
            if not self._t0 - self._t0_moe <= t <= self._tf:
                raise Exception('Time {} should be between {} and {}'.format(t, self._t0, self._tf))
            elif self._t0 - self._t0_moe <= t < self._t0:
                print 'Warning, t: {} < t0: {} but within Margin of Error: {}'.format(t, self._t0, self._t0_moe)
                t = np.array(self._t0)

        if not self._dx.shape[0] == t.size:
            self._dx = np.zeros([t.size, self._dimensions])

        # Since all interpolation is done before 0.0 and tf, we adjust t accordingly by subtracting self._t0
        t = t - self._t0
        for i in range(0, self._dimensions):
            c = self._coefficients[:, i]
            self._dx[:, i] = c[1] + 2 * c[2] * t + 3 * c[3] * (t**2) + 4 * c[4] * (t**3) + 5 * c[5] * (t**4)
        self._lock.release()
        return self._dx

    def get_interpolated_ddx(self, t):
        self._lock.acquire()

        if not type(t) is np.array:
            t = np.array(t)
        # t = np.array(t)
        if t.size == 1:
            if not self._t0 - self._t0_moe <= t <= self._tf:
                raise Exception('Time {} should be between {} and {}'.format(t, self._t0, self._tf))
            elif self._t0 - self._t0_moe <= t < self._t0:
                print 'Warning, t: {} < t0: {} but within Margin of Error: {}'.format(t, self._t0, self._t0_moe)
                t = np.array(self._t0)

        if not self._ddx.shape[0] == t.size:
            self._ddx = np.zeros([t.size, self._dimensions])

        # Since all interpolation is done before 0.0 and tf, we adjust t accordingly by subtracting self._t0
        t = t - self._t0
        for i in range(0, self._dimensions):
            c = self._coefficients[:, i]
            self._ddx[:, i] = 2 * c[2] + 6 * c[3] * t + 12 * c[4] * (t**2) + 20 * c[5] * (t**3)
        self._lock.release()
        return self._ddx

    def get_t0(self):
        return self._t0

    def get_tf(self):
        return self._tf

    def compute_interpolation_params(self, x0, xf, dx0, dxf, ddx0, ddxf, t0, tf):
        self._lock.acquire()
        if not isinstance(x0, np.ndarray):
            x0 = np.asarray(x0)
        if not isinstance(xf, np.ndarray):
            xf = np.asarray(xf)
        if not isinstance(dx0, np.ndarray):
            dx0 = np.asarray(dx0)
        if not isinstance(dxf, np.ndarray):
            dxf = np.asarray(dxf)
        if not isinstance(ddx0, np.ndarray):
            ddx0 = np.asarray(ddx0)
        if not isinstance(ddxf, np.ndarray):
            ddxf = np.asarray(ddxf)
        if x0.size != xf.size != dx0.size != dxf.size != ddx0.size != ddxf.size:
            raise Exception('All arrays for initial and final P,V & A must be of same length')

        self._dimensions = x0.size

        if not self._x.shape[1] == self._dimensions:
            self._x = np.zeros([1, self._dimensions])
            self._dx = np.zeros([1, self._dimensions])
            self._ddx = np.zeros([1, self._dimensions])

        if not self._boundary_conditions.shape[0] == self._dimensions:
            self._boundary_conditions = np.zeros([self._dimensions, 6])
            self._coefficients = np.zeros([6, self._dimensions])
            self._T_mat = np.zeros([6, 6, self._dimensions])

        self._t0 = t0
        self._tf = tf

        t0_adjusted = 0.0
        tf_adjusted = tf - t0
        if tf_adjusted <= 0:
            raise Exception('tf: {} cannot be less than t0: {}'.format(tf, t0))

        for i in range(0, self._dimensions):
            self._boundary_conditions[i, :] = np.mat([x0[i], dx0[i], ddx0[i], xf[i], dxf[i], ddxf[i]])
            self._T_mat[:, :, i] = self._compute_time_mat(t0_adjusted, tf_adjusted)
            self._coefficients[:, i] = np.matmul(np.linalg.inv(self._T_mat[:, :, i]),
                                                 np.transpose(self._boundary_conditions[i, :]))
        self._lock.release()

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
