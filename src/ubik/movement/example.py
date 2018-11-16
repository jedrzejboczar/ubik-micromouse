import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

data_lin = pd.read_csv('sim_lin.csv')
data_ang = pd.read_csv('sim_ang.csv')

data_lin.plot(grid=True, x='time')
plt.show()
data_ang.plot(grid=True, x='time')
plt.show()
