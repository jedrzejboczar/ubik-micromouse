#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt


def cosine_lookup_table(length):
    table = np.cos(np.linspace(0, np.pi/2, length))
    return table


pi = np.pi
two_pi = 2*pi
half_pi = pi/2


#  def cos_1q(x):
#      cosine_lookup = cosine_lookup_table(N)
#      index_float = x / half_pi * (N-2)
#      value = cosine_lookup[int(index_float)]
#      return value

def cos_1q(x, cosine_lookup):
    N = len(cosine_lookup)
    index_float = x / half_pi * (N-2)
    index_low = int(index_float)
    index_high = int(index_float+1)
    cos_low = cosine_lookup[index_low]
    cos_high = cosine_lookup[index_high]
    # a = dy/dx,   1 cause it is by index
    a = (cos_high - cos_low) / 1
    value = cos_low + a * (index_float - index_low)
    return value

#  # or from taylor series
#  def cos_1q(x, _):
#      import math
#      fac = math.factorial
#      return 1 - x**2/fac(2) + x**4/fac(4)   - x**6 / fac(6)   #+ x**8 / fac(8)

def cos(x, cosine_lookup):
    # modulo to range [0, 2pi]
    x = x % two_pi
    # cos(-x) = cos(x)
    if x < 0:
        x = -x
    # check in which quadrant we are
    quadrant = int(x / half_pi)
    # use trigonometric identites to convert x to 1 quadrant
    if quadrant == 0:
        return cos_1q(x, cosine_lookup)
    elif quadrant == 1:
        return -cos_1q(pi - x, cosine_lookup)
    elif quadrant == 2:
        return -cos_1q(x - pi, cosine_lookup)
    elif quadrant == 3:
        return cos_1q(two_pi - x, cosine_lookup)
    assert 0, "Something is not yes"

def sin(x, cosine_lookup):
    return cos(x - half_pi, cosine_lookup)

def test_error(x, cosine_lookup):
    cos_ref = np.cos(x)
    cos_approx = np.array([cos(xi, cosine_lookup) for xi in x])
    error = cos_ref - cos_approx
    mse = np.sum(error**2) / len(error)
    return np.max(np.abs(error)), mse

def plot_test_error(N, cosine_lookup):
    print('dx = %f' % (x[1] - x[0]))
    error_only = True

    if 1:
        plt.figure()
        sin_ref = np.sin(x)
        sin_approx = np.array([sin(xi, cosine_lookup) for xi in x])
        error = sin_ref - sin_approx
        mse = np.sum(error**2) / len(error)
        print('Mean square error = %e' % mse)
        if error_only:
            plt.plot(x, error / 1 * 100, label='error [%]')
        else:
            plt.plot(x, sin_ref, label='sin_ref(x)')
            plt.plot(x, sin_approx, label='sin_approx(x)')
            plt.plot(x, error, label='sin_ref(x) - sin_approx(x)')
        plt.xlabel('x')
        plt.ylabel('f(x)')
        plt.grid(True)
        plt.legend()

    if 1:
        plt.figure()
        cos_ref = np.cos(x)
        cos_approx = np.array([cos(xi, cosine_lookup) for xi in x])
        error = cos_ref - cos_approx
        mse = np.sum(error**2) / len(error)
        print('Mean square error = %e' % mse)
        if error_only:
            plt.plot(x, error / 1 * 100, label='error [%]')
        else:
            plt.plot(x, cos_ref, label='cos_ref(x)')
            plt.plot(x, cos_approx, label='cos_approx(x)')
            plt.plot(x, error, label='cos_ref(x) - cos_approx(x)')
        plt.xlabel('x')
        plt.ylabel('f(x)')
        plt.grid(True)
        plt.legend()

    plt.show()

if __name__ == '__main__':
    x = np.linspace(0, 2*np.pi, 100000)

    if 0:
        cosine_lookup = cosine_lookup_table(1000)
        plot_test_error(x, cosine_lookup)
    else:
        max_errors = []
        mse_values = []
        #  n_range = np.arange(100, 500, 50)
        n_range = np.logspace(2, 3, 20)
        for n in n_range:
            print('n = %d' % n)
            cosine_lookup = cosine_lookup_table(n)
            max_err, mse = test_error(x, cosine_lookup)
            max_errors.append(max_err)
            mse_values.append(mse)
        fig, ax = plt.subplots(2)
        ax[0].plot(n_range, max_errors, '.-', label='Max error(N)')
        ax[0].grid(True)
        ax[0].legend()
        ax[1].plot(n_range, mse_values, '.-', label='Mean square error(N)')
        ax[1].grid(True)
        ax[1].legend()
        plt.show()

