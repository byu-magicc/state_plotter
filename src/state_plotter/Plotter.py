#!/usr/bin/env python
from threading import Lock
import numpy as np
import pyqtgraph as pg
from pyqtgraph import ViewBox
import argparse

# Enable antialiasing for prettier plots
pg.setConfigOptions(antialias=True)

class Plotter:
    """
    Class for plotting methods.
    """
    def __init__(self, plotting_frequency=1, time_window=15):
        ''' Initialize the Plotter

            plotting_freq: number of times the update function must be called
                           until the plotter actually outputs the graph.
                           (Can help reduce the slow-down caused by plotting)

            time_window:   how many seconds of data to appear in the plot
        '''
        self.time_window = time_window
        self.time = 0
        self.prev_time = 0

        # Able to update plots intermittently for speed
        self.plotting_frequency = plotting_frequency
        self.freq_counter = 0

        # Plot default parameters
        self.plots_per_row = 3
        self.x_grid_on = False
        self.y_grid_on = True
        self.default_label_pos = 'left'
        self.auto_adjust_y = True
        # Plot color parameters
        self.distinct_plot_hues = 3 # Number of distinct hues to cycle through
        self.plot_min_hue = 0
        self.plot_max_hue = 270
        self.plot_min_value = 200
        self.plot_max_value = 255
        # Plot theme params -- default is dark theme
        self.background_color = 'k'
        self.axis_pen = pg.mkPen(color='w', width=1)

        # initialize Qt gui application and window
        self.default_window_size = (1000, 800)
        self.app = pg.QtGui.QApplication([])
        self.window = pg.GraphicsWindow(title="States")
        self.window.resize(*self.default_window_size)
        self.window.setBackground(self.background_color)
        self.old_windows = []

        # Define plot argument parser
        self.arg_parser = self._define_plot_arg_parser()

        self.plot_cnt = 0
        self.plots = {}
        self.curves = {}
        self.curve_colors = {}
        self.states = {}
        self.input_vectors = {}
        self.new_data = False
        self.states_lock = Lock() # Lock to prevent asynchronous changes to states


    def define_input_vector(self, vector_name, input_vector):
        ''' Defines an input vector so measurements can be added in groups

            vector_name (string): name of the vector
            input_vector (list of strings): order of states in the vector

            Note: this does not add states or plots, so plots for the values in
            *input_vector* will need to also be added via the *add_plot* function
        '''
        self.input_vectors[vector_name] = input_vector

    def add_window(self, window_title):
        # Create a new window
        self.window = pg.GraphicsWindow(title=window_title)
        self.window.resize(*self.default_window_size)
        self.window.setBackground(self.background_color)

        # Reset plot count
        self.plot_cnt = 0

    def use_light_theme(self):
        self.background_color = 'w'
        self.window.setBackground(self.background_color)
        self.axis_pen = pg.mkPen(color='k', width=1)
        self.plot_min_hue = 360
        self.plot_max_hue = 72
        self.plot_min_value = 0
        self.plot_max_value = 180

    def set_plots_per_row(self, n):
        self.plots_per_row = n

    def set_grids(self, x_grid_on, y_grid_on):
        self.x_grid_on = x_grid_on
        self.y_grid_on = y_grid_on

    def add_plot(self, plot_str):
        ''' Adds a state and the necessary plot, curves, and data lists

            curve_names: name(s) of the state(s) to be plotted in the same plot window
                         (e.g. ['x', 'x_truth'] or ['x', 'x_command'])
        '''
        # Parse the string for curve names and arguments
        plot_args = self._parse_plot_str(plot_str)

        plot_name = plot_args.name
        self._add_plot_box(plot_name, include_legend=plot_args.legend)

        # Add each curve to the plot
        curve_color_idx = 0
        for curve in plot_args.curves:
            self._add_curve(plot_name, curve, curve_color_idx)
            curve_color_idx += 1

    def add_vector_measurement(self, vector_name, vector_values, time, rad2deg=False):
        '''Adds a group of measurements in vector form

            vector_name (string): name given the vector through the *define_input_vector*
                                  function
            vector_values (list of numbers): values for each of the states in the
                          order defined in the *define_input_vector* function
            time: time stamp for the values in the vector
            rad2deg: Flag to convert the state value from radians to degrees

        '''
        state_index = 0
        if len(vector_values) != len(self.input_vectors[vector_name]):
            print("ERROR: State vector length mismatch. \
                          State vector '{0}' has length {1}".format(vector_name, len(vector_values)))
        for state in self.input_vectors[vector_name]:
            self.add_measurement(state, vector_values[state_index], time, rad2deg=rad2deg)
            state_index += 1


    def add_measurement(self, state_name, state_val, time, rad2deg=False):
        '''Adds a measurement for the given state

            state_name (string): name of the state
            state_val (number): value to be added for the state
            time (number): time (in seconds) of the measurement
            rad2deg: Flag to convert the state value from radians to degrees
        '''
        if rad2deg:
            state_val *= 180.0/np.pi
        self.states_lock.acquire()
        self.states[state_name][0].append(time)
        self.states[state_name][1].append(state_val)
        self.states_lock.release()
        self.new_data = True
        if time > self.time:
            self.time = time # Keep track of the latest data point


    # Update the plots with the current data
    def update_plots(self):
        '''Updates the plots (according to plotting frequency defined in initialization) '''

        if self.time > self.prev_time:
            # Only process data if time has changed
            self.freq_counter += 1
            if self.new_data and (self.freq_counter % self.plotting_frequency == 0):

                for curve in self.curves:
                    data = self.states[curve]

                    # If there is no data, just skip for now
                    if not data:
                        continue

                    self.states_lock.acquire()
                    time_array = data[0]
                    values_array = data[1]
                    self.curves[curve].setData(time_array, values_array, pen=self.curve_colors[curve])
                    self.states_lock.release()

                x_min = max(self.time - self.time_window, 0)
                x_max = self.time
                for plot in self.plots:
                    self.plots[plot].setXRange(x_min, x_max)
                    self.plots[plot].enableAutoRange(axis=ViewBox.YAxis)

                self.new_data = False
                self.prev_time = self.time

        # update the plots
        self.app.processEvents()


    #
    # Private Methods
    #

    def _get_color(self, index):
        ''' Returns incremental plot colors based on index '''
        return pg.intColor(index, minValue=self.plot_min_value, maxValue=self.plot_max_value,
                            hues=self.distinct_plot_hues, minHue=self.plot_min_hue, maxHue=self.plot_max_hue)


    def _add_plot_box(self, plot_name, include_legend=False):
        ''' Adds a plot box to the plotting window '''
        if len(self.plots) % self.plots_per_row == 0:
            self.window.nextRow()
        self.plots[plot_name] = self.window.addPlot()
        self.plots[plot_name].setLabel(self.default_label_pos, plot_name)
        if include_legend:
            self._add_legend(plot_name)
        if self.auto_adjust_y:
            state = self.plots[plot_name].getViewBox().getState()
            state["autoVisibleOnly"] = [False, True]
            self.plots[plot_name].getViewBox().setState(state)
            self.plots[plot_name].getAxis("bottom").setPen(self.axis_pen)
            self.plots[plot_name].getAxis("left").setPen(self.axis_pen)

    def _add_legend(self, plot_name):
        self.plots[plot_name].addLegend(size=(1,1), offset=(1,1))

    def _add_curve(self, plot_name, curve_name, curve_color_idx=0):
        ''' Adds a curve to the specified plot

            plot_name: Name of the plot the curve will be added to
            curve_name: Name the curve will be referred by
            curve_color_idx: index of the curve in the given plot
                       (i.e. 0 if it's the first curve, 1 if it's the second, etc.)
                       Used to determine the curve color with *_get_color* function
        '''
        curve_color = self._get_color(curve_color_idx)
        self.curves[curve_name] = self.plots[plot_name].plot(name=curve_name)
        self.curve_colors[curve_name] = curve_color
        self.states[curve_name] = [[],[]]

    def _define_plot_arg_parser(self):
        parser = argparse.ArgumentParser()
        parser.add_argument("curves", nargs="+")
        parser.add_argument('-l', '--legend', action='store_true')
        parser.add_argument('-n', '--name', nargs="+")

        dim_group = parser.add_mutually_exclusive_group()
        dim_group.add_argument('-2d', '--2d', action='store_const', dest="dimension", const=2, default=1)
        dim_group.add_argument('-3d', '--3d', action='store_const', dest="dimension", const=3, default=1)
        return parser

    def _parse_plot_str(self, plot_str):
        args = self.arg_parser.parse_args(plot_str.split())

        # Find hidden curves
        args.hidden_curves = []
        for c in args.curves:
            if c[0] == "_":
                args.hidden_curves.append(c[1:])
        # Remove from regular curves
        for c in args.hidden_curves:
            args.curves.remove("_" + c)

        # Check for dimension issues
        if args.dimension > len(args.curves):
            e = "Plot string error: dimension ({0}) does not match number of curves ({1}).".format(args.dimension, args.curves)
            raise Exception(e)

        if args.name is None:
            args.name = args.curves[0]

        return args
