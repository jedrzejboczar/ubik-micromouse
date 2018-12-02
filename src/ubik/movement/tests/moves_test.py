import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd

from colorline import colorline

data = pd.read_csv('sim_moves.csv')
#  print(data)

fig, axes = plt.subplots(2, gridspec_kw=dict(height_ratios=[5, 2]))
data.plot(ax=axes[0], grid=True, x='x', y='y', label='position (x, y)')
axes[0].set_adjustable('datalim')
axes[0].set_aspect('equal')
data.plot(ax=axes[1], grid=True, x='t', y='theta', label='theta(t)')

if 0:  # (x, y) with coloring based on time
    fig, ax = plt.subplots()
    # colormaps: viridis, inferno, plasma, magma
    cmap = plt.get_cmap('inferno')
    norm = plt.Normalize(data['t'].iloc[0], data['t'].iloc[-1])
    colorline(data['x'].values, data['y'].values, data['t'].values,
              cmap=cmap, norm=norm, linewidth=5)
    dx = np.abs(data['x'].max() - data['x'].min()) / 5
    dy = np.abs(data['y'].max() - data['y'].min()) / 5
    ax.set_xlim(data['x'].min() - dx, data['x'].max() + dx)
    ax.set_ylim(data['y'].min() - dy, data['y'].max() + dy)
    ax.set_adjustable('datalim')
    ax.set_aspect('equal')
    ax.grid(True)
    cax = fig.add_axes([0.95, 0.15, 0.02, 0.7])
    matplotlib.colorbar.ColorbarBase(cax, cmap=cmap, norm=norm)
    fig.tight_layout()

plt.show()

