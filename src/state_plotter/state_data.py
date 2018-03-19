#!/usr/bin/env python

class StateData():
    def __init__(self, is_angle=False, use_degrees=False):
        self.data = []
        self.time = []
        self.is_angle = is_angle
        self.use_degrees = use_degrees
