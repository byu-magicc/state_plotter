#!/usr/bin/env python
import time
from builtins import input

import numpy as np

from state_plotter.Plotter import Plotter

# For the sake of testing
ADD_BY_VECTOR = False

plot = Plotter()

# Define plot names
#   - Test multiple plots
#   - Test adding legend (-l)
#   - Test hiding plots (prefix "_")
#   - Test 2D plot (-2d)
plots =  ['x x_truth -l',      'y y_truth',        'z z_truth',
         'phi _phi_e',     'theta _theta_e',    'psi _psi_e',
         'x y x_truth y_truth -2d'
        ]

# Add plots to the window
for p in plots:
    plot.add_plot(p)

plot.define_input_vector('position', ['x', 'y', 'z'])
plot.define_input_vector('true_position', ['x_truth', 'y_truth', 'z_truth'])
plot.define_input_vector('attitude', ['phi', 'theta', 'psi'])
plot.define_input_vector('estimated_attitude', ['phi_e', 'theta_e', 'psi_e'])


# setup simulation timing
T = 5
Ts = 0.01
tvec = np.linspace(0, T, num=int((1/Ts)*T))

# run the simulation
for idx, t in enumerate(tvec):
    # Make some sines and stuff
    x = 10*np.sin(2*np.pi*1*t)
    y = 10*np.cos(2*np.pi*0.5*t)
    z = 10*np.tan(2*np.pi*2*t)

    x_t = 15*np.sin(2*np.pi*1*t)
    y_t = 15*np.cos(2*np.pi*0.5*t)
    z_t = 15*np.tan(2*np.pi*2*t)

    phi = np.sin(2*np.pi*1*t)
    theta = np.cos(2*np.pi*0.5*t)
    psi = np.tan(2*np.pi*2*t)

    phi_e = 1.5*np.sin(2*np.pi*1*t)
    theta_e = 1.5*np.cos(2*np.pi*0.5*t)
    psi_e = 1.5*np.tan(2*np.pi*2*t)

    if ADD_BY_VECTOR:
        plot.add_vector_measurement('position', [x, y, z], t)
    else:
        plot.add_measurement('x', x, t)
        plot.add_measurement('y', y, t)

        # Demonstrate plotting with independent measurement intervals
        if np.mod(idx, 10) == 0:
            plot.add_measurement('z', z, t)

    # Add the rest of the states
    plot.add_vector_measurement('true_position', [x_t, y_t, z_t], t)
    plot.add_vector_measurement('attitude', [phi, theta, psi], t)
    plot.add_vector_measurement('estimated_attitude', [phi_e, theta_e, psi_e], t)

    # Update and display the plot
    plot.update_plots()

# Wait so that the plot doesn't disappear
input("Press any key to end...")
