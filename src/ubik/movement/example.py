import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

data = pd.read_csv('sim.csv')

data.plot(grid=True, x='time')
plt.show()
