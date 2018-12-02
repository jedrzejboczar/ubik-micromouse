import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

data = pd.read_csv('sim.csv')
numeric_columns = [col for col in data.columns if col not in ['time', 'phase']]

fig, axes = plt.subplots(2, sharex=True, gridspec_kw=dict(height_ratios=[5, 2]))
data.plot(x='time', y=numeric_columns, ax=axes[0], grid=True)
#  data.plot(x='time', y='phase', ax=axes.twinx(), color='C%d' % (len(numeric_columns)), grid=True)
data.plot(x='time', y='phase', ax=axes[1], color='C%d' % (len(numeric_columns)), grid=True)

#  print(data)

plt.show()
