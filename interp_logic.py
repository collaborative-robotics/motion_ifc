import numpy as np
import scipy.linalg as la
import matplotlib.pyplot as plt


class Interpolation(object):
    def __init__(self):
        pass

    def _compute_time_mat(self, t0, tf):
        T_mat = np.mat([[1, t0, t0 ** 2, t0 ** 3, t0 ** 4, t0 ** 5],
                        [0, 1, 2 * t0, 3 * t0 ** 2, 4 * t0 ** 3, 5 * t0 ** 4],
                        [0, 0, 2, 6 * t0, 12 * t0 ** 2, 20 * t0 ** 3],
                        [1, tf, tf ** 2, tf ** 3, tf ** 4, tf ** 5],
                        [0, 1, 2 * tf, 3 * tf ** 2, 4 * tf ** 3, 5 * tf ** 4],
                        [0, 0, 2, 6 * tf, 12 * tf ** 2, 20 * tf ** 3]])
        return T_mat

    def get_interpolated_trajectory(self, t, coeff):
        pos = coeff[0] + coeff[1] * t + coeff[2] * (t**2) + coeff[3] * (t**3) + coeff[4] * (t**4) + coeff[5] * (t**5)
        vel = coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * (t**2) + 4 * coeff[4] * (t**3) + 5 * coeff[5] * (t**4)
        acc = 2 * coeff[2] + 6 * coeff[3] * t + 12 * coeff[4] * (t**2) + 20 * coeff[5] * (t**3)
        return [pos, vel, acc]

    def interpolate(self, p0, pf, v0, vf, a0, af, t0, tf):
        dims = p0.__len__()
        print 'Number of Dimensions = {}'.format(dims)
        if p0.__len__() != pf.__len__() or v0.__len__() != vf.__len__() or a0.__len__() != af.__len__():
            raise Exception('All arrays for initial and final P,V & A must be of same length')

        n = 50

        c = np.zeros([dims, 6])
        coeff = np.zeros([6, dims])
        T_mat = np.zeros([6, 6, dims])

        pos = np.zeros([n, dims])
        vel = np.zeros([n, dims])
        acc = np.zeros([n, dims])

        t = np.linspace(t0, tf, n)

        for i in range(0, dims):
            c[i, :] = np.mat([p0[i], v0[i], a0[i], pf[i], vf[i], af[i]])
            T_mat[:, :, i] = self._compute_time_mat(t0, tf)
            coeff[:, i] = np.matmul(np.linalg.inv(T_mat[:, :, i]), np.transpose(c[i, :]))
            [pos[:, i], vel[:, i], acc[:, i]] = self.get_interpolated_trajectory(t, coeff[:, i])
        plt.plot(t, pos, 'o', t, vel, '-', t, acc, '--')
        plt.legend([['position'], 'velocity', 'acceleration'], loc='best')
        plt.show()


obj = Interpolation()
obj.interpolate([ 0.5, 1.0,-0.3],
                [-0.5, 0.0, 0.2],
                [ 0.0, 0.0,-0.2],
                [ 0.5, 0.0, 0.0],
                [ 0.0, 0.0, 0.0],
                [-0.3, 0.0, 0.0],
                  0.0, 10.0)