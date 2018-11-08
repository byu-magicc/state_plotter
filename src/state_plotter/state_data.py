#!/usr/bin/env python
import numpy as np

class StateData():
    def __init__(self, max_length=None, is_angle=False, rad2deg=False):
        self.data = []
        self.time = []
        self.max_length = max_length
        self.is_angle = is_angle
        self.rad2deg = rad2deg

    def add_data(self, data, t):
        if self.is_angle:
            data = angle_wrap(data)
        if self.rad2deg:
            data *= 180.0/np.pi
        self.data.append(data)
        self.time.append(t)
        if self.max_length is not None and len(self.data) > self.max_length:
            self.pop(0)

    def get_data_vec(self):
        return self.data

    def get_time_vec(self):
        return self.time

    def pop(self, idx=-1):
        self.data.pop(idx)
        self.time.pop(idx)

def angle_wrap(x):
    xwrap = np.array(np.mod(x, 2*np.pi))
    mask = np.abs(xwrap) > np.pi
    xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
    if np.size(xwrap) == 1:
        return float(xwrap)
    else:
        return xwrap
