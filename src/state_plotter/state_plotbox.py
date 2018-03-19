#!/usr/bin/env python
import pyqtgraph as pg

class StatePlotbox():
    def __init__(self, window, dimension=1, legend=False, axis_color='w', axis_width=1):
        # Initlialize plotbox
        self.plotbox = window.addPlot()

        # Store parameters
        self.dimension = dimension
        self.set_axis_color(axis_color, axis_width)

        if legend:
            self.add_legend()

    def label_axes(self, x_label=None, y_label=None):
        if x_label is not None:
            self.plotbox.getAxis("bottom").setPen(self.axis_pen)

    def set_axis_color(self, color, width=1):
        self.axis_pen = pg.mkPen(color=axis_color, width=axis_width)

    def add_legend(self):
