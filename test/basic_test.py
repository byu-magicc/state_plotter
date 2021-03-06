#!/usr/bin/env python
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.getcwd()), 'src'))

import time
from builtins import input
from IPython.core.debugger import set_trace

import numpy as np

from state_plotter.Plotter import Plotter
from state_plotter.plotter_args import *

plotter = Plotter(plotting_frequency=1)

### Define plot names

## Simple string definitions
x_plot = 'x'
y_plot = 'y'
z_plot = 'z'

## Multiple plots in a plotbox (using PlotboxArgs)
# -Simply add multiple plot names
phi_plot = PlotboxArgs(plots=['phi', 'phi_e'])
# -Add title to the plotbox
theta_plot = PlotboxArgs(
    title="Multiple theta plots",
    plots=['theta', 'theta_e']
)
# -Use Plot args to name different curves in the legend,
# -Use 'labels' to get more detailed x and y labels
# -Use rad2deg to automatically wrap angles and convert radians to degrees
psi_plot = PlotboxArgs(
    title="Multiple psi plots",
    plots=[PlotArgs('True psi', states=['psi']),
           PlotArgs('Estimated psi', states='psi_e')],
    labels={'left':'Psi (deg)', 'bottom':'Time (s)'},
    rad2deg=True
)

## Two dimensional plots
# -Simple 2D plot. Use PlotArgs to combine two states into a 2D plot
xy_plot = PlotboxArgs(
    title="XY Plane",
    plots=[PlotArgs(states=['x', 'y']),
           PlotArgs(states=['x_e', 'y_e'])]
)
# -Add names to the different 2D curves
xz_plot = PlotboxArgs(
    title="XZ plane",
    plots=[PlotArgs('True xz position', states=['x', 'z']),
           PlotArgs('Estimated xz position', states=['x_e', 'z_e'])]
)
# -Add extra labels to the plotbox for clarity
# -Use max_length to only plot the last 100 data points
yz_plot = PlotboxArgs(
    title="YZ plane",
    plots=[PlotArgs('True yz position', states=['y', 'z']),
           PlotArgs('Estimated yz position', states=['y_e', 'z_e'])],
    labels={'bottom':'Y Position (m)', 'left':'Z Position (m)'},
    max_length=100
)

# Use a list of lists to specify plotting window structure (3 rows, each with 3 plots)

first_row = [x_plot, y_plot, z_plot]
second_row = [phi_plot, theta_plot, psi_plot]
third_row = [xy_plot, xz_plot, yz_plot]

plots = [first_row,
         second_row,
         third_row
        ]

# Add plots to the window
plotter.add_plotboxes(plots)

# Define and label vectors for more convenient/natural data input
plotter.define_input_vector('position', ['x', 'y', 'z'])
plotter.define_input_vector('estimated_position', ['x_e', 'y_e', 'z_e'])
plotter.define_input_vector('attitude', ['phi', 'theta', 'psi'])
plotter.define_input_vector('estimated_attitude', ['phi_e', 'theta_e', 'psi_e'])


# setup simulation timing
T = 5
Ts = 0.01
tvec = np.linspace(0, T, num=int((1/Ts)*T))

# run the simulation
for idx, t in enumerate(tvec):
    # Make some sinusoids and stuff
    x = np.sin(2*np.pi*1*t)
    y = np.cos(2*np.pi*0.5*t)
    z = t + np.cos(2*np.pi*2*t)

    x_t = 1.5*np.sin(2*np.pi*1*t)
    y_t = 1.5*np.cos(2*np.pi*0.5*t)
    z_t = t

    phi = np.sin(2*np.pi*1*t)
    theta = np.cos(2*np.pi*0.5*t)
    psi = t

    phi_e = 1.5*np.sin(2*np.pi*1*t)
    theta_e = 1.5*np.cos(2*np.pi*0.5*t)
    psi_e = t + 0.1*np.cos(2*np.pi*2*t)

    ## Add the state data in vectors
    plotter.add_vector_measurement('position', [x, y, z], t)
    plotter.add_vector_measurement('estimated_position', [x_t, y_t, z_t], t)
    plotter.add_vector_measurement('attitude', [phi, theta, psi], t)
    # Demonstrate plotting with independent measurement intervals
    if np.mod(idx, 3) == 0:
        plotter.add_vector_measurement('estimated_attitude', [phi_e, theta_e, psi_e], t)

    # Update and display the plot
    plotter.update_plots()
    time.sleep(0.015)

# Wait so that the plot doesn't disappear
input("Press any key to end...")
