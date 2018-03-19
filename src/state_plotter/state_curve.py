#!/usr/bin/env python

class StateCurve():
    def __init__(self, curve_name, plotbox, color, dimension=1, hidden=False):
        # Initialize the pyqtgraph plot curve
        self.name = curve_name
        self.curve = plotbox.plot(name=self.name)

        # Collect params
        self.color = color
        self.dimension = dimension
        self.hidden = hidden

        # Initialize states
        self.states = {}

    def add_state(self, name, is_angle=False, use_degrees=False):
        self.states[name] = StateData(is_angle=is_angle, use_degrees=use_degrees)

    
