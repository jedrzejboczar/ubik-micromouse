import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

data_lin = pd.read_csv('sim_lin.csv')
data_ang = pd.read_csv('sim_ang.csv')
data_pos = pd.read_csv('sim_pos.csv')


# movement velocities of the profiler
fig, axes = plt.subplots(2)
data_lin.plot(ax=axes[0], grid=True, x='time')
data_ang.plot(ax=axes[1], grid=True, x='time')


# movement simulation
fig, axes = plt.subplots()
data_pos.plot(ax=axes, grid=True, x='x', y='y', label='position (x, y)').set_aspect('equal')

#  fig, axes = plt.subplots(2)
#  axes[0].plot(data_pos['time'], data_pos['theta'], label='theta(t)')
#  axes[1].plot(data_pos['time'], data_pos['theta'].diff(), label='theta_dot(t)')
#  axes[0].grid(True)
#  axes[1].grid(True)
#  axes[0].legend()
#  axes[1].legend()


plt.show()
