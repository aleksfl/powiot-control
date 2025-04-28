# -*- coding: utf-8 -*-
"""
This module implements the baseline control for testcases.

"""

class BaselineController:
    def __init__(self):
        pass  # No specific initialization required

    def compute_control(self, y, time, forecasts=None):        
        u = {}  # No control actions in baseline
        return u

    def initialize(self):        
        u = {}
        return u
