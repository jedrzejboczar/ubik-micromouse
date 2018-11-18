import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

data_lin = pd.read_csv('sim_lin.csv')
data_ang = pd.read_csv('sim_ang.csv')

fig, axes = plt.subplots(2)

data_lin.plot(ax=axes[0], grid=True, x='time')
data_ang.plot(ax=axes[1], grid=True, x='time')
plt.show()
