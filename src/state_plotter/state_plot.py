#!/usr/bin/env python
from collections import OrderedDict
from plotter_args import PlotArgs
from state_data import StateData
import numpy as np
from pdb import set_trace

class StatePlot():
    def __init__(self, plotbox, args):
        if not isinstance(args, PlotArgs):
            raise TypeError('\'args\' argument must be of type PlotArgs')
        # Initialize the pyqtgraph plot plot
        self.name = args.name
        self.plotbox = plotbox
        self.hidden = args.hidden

        if not self.hidden:
            self.plot = self.plotbox.plot(name=self.name)

        # Collect params
        self.color = args.color
        self.connect = args.connect
        self.symbol = args.symbol
        self.symbol_size = args.symbol_size
        self.px_mode = args.px_mode
        self.dimension = len(args.state_names)
        self.marker = None
        self.marker_scale = 0.04 # Percentage of the minimum plotbox dimension for the circle radius
        if self.dimension == 2:
            self.marker = plotbox.plot()
            self.xy_marker_circle = self._get_ellipse((0,0), radius=1.0)

        # Initialize states
        self.states = OrderedDict()

        for name in args.state_names:
            self.add_state(name, args.max_length, args.is_angle, args.rad2deg)

    def add_state(self, name, max_length=None, is_angle=False, rad2deg=False):
        self.states[name] = StateData(max_length=max_length, is_angle=is_angle, rad2deg=rad2deg)

    def get_states(self):
        return self.states

    def update(self):
        if self.hidden:
            return
        # Get data from state objects
        state_objs = self.states.values()
        if self.dimension == 1:
            x_data = state_objs[0].get_time_vec()
            y_data = state_objs[0].get_data_vec()
        elif self.dimension == 2:
            x_data = state_objs[0].get_data_vec()
            y_data = state_objs[1].get_data_vec()
        else:
            raise NotImplementedError('Plots with dimension > 2 are not yet supported.')


        # Update the data for the plot (and marker, if necessary)
        if not self.connect:
            self.plot.setData(x_data, y_data, pen=None, symbol=self.symbol,
                               symbolSize=self.symbol_size, symbolPen=self.color, pxMode=self.px_mode)
        else:
            self.plot.setData(x_data, y_data, pen=self.color)
        if self.marker is not None and len(x_data) > 0 and len(y_data) > 0:
            x_range = self.plotbox.vb.targetRange()[0]
            y_range = self.plotbox.vb.targetRange()[1]
            scale = self.marker_scale*min(x_range[1]-x_range[0], y_range[1]-y_range[0])
            # scale = 10.0
            marker = scale*self.xy_marker_circle + np.array([[x_data[-1]], [y_data[-1]]])
            self.marker.setData(marker[0], marker[1], pen=self.color)


    def _get_ellipse(self, center, radius):
        N = 100
        theta = np.linspace(0, 2*np.pi, N)
        if isinstance(radius, float) or isinstance(radius, int):
            radius = [radius, radius]
        x = np.cos(theta)*radius[0] + center[0]
        y = np.sin(theta)*radius[1] + center[1]
        return np.array([x,y])
