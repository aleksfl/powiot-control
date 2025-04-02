# -*- coding: utf-8 -*-
"""
This module implements the baseline control for testcases.

"""

class BaselineController:
    def __init__(self):
        pass  # No specific initialization required

    def compute_control(self, y):        
        u = {}  # No control actions in baseline
        return u, 0  # Also returning 0 as error (since it’s a baseline)

    def initialize(self):        
        u = {}
        return u
