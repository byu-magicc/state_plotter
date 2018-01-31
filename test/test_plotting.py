#!/usr/bin/env python
import time
from builtins import input

import numpy as np

from state_plotter.Plotter import Plotter

plot = Plotter()

plot.define_state_vector('MAV', ['x', 'y', 'z'])

plot.add_plot('x', include_legend=True)
plot.add_plot('y', include_legend=False)
plot.add_plot('z', include_legend=False)

# setup simulation timing
T = 5
Ts = 0.01
tvec = np.linspace(0, T, num=int((1/Ts)*T))

# run the simulation
for t in tvec:
    # Make some sines and stuff
    x = np.sin(2*np.pi*1*t)
    y = np.cos(2*np.pi*0.5*t)
    z = np.tan(2*np.pi*2*t)

    plot.add_vector_measurement('MAV', [x, y, z], t)

    # Update and display the plot
    plot.update_plots()

# Wait so that the plot doesn't disappear
input("Press any key to end...")