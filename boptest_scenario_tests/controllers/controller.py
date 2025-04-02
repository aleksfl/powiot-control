# -*- coding: utf-8 -*-
"""
This module contains a generic controller class that is used to instantiate
concrete controllers, found in baseline.py, pi.py, and mpc.py.

"""

import sys
import importlib


class Controller(object):
    def __init__(self, module, use_forecast=False):
        """Controller object that instantiates concrete controller methods.

        Parameters
        ----------
        module : str
            path to concrete implementation of controller
        use_forecast : bool
            True if controller uses forecasts.
            Default is False.

        """

        try:
            # instantiate the concrete controller specified in the configuration
            controller_module = importlib.import_module(module)
        except ModuleNotFoundError:
            print("Cannot find specified controller: {}".format(module))
            sys.exit()

        # Automatically determine the expected class name
        controller_class_name = f"{module.split('.')[1].upper() if len(module.split('.')[1]) <= 3 else module.split('.')[1].capitalize()}Controller"

        if not hasattr(controller_module, controller_class_name):
            print(f"Module {module} does not define class {controller_class_name}.")
            sys.exit()

        # Instantiate the appropriate controller class
        self.controller = getattr(controller_module, controller_class_name)()

        if use_forecast:
            self.use_forecast = True
            self.update_forecasts = getattr(self.controller, "update_forecasts", None)
            self.get_forecast_parameters = getattr(self.controller, "get_forecast_parameters", None)
        else:
            self.use_forecast = False
        self.compute_control = self.controller.compute_control
        self.initialize = self.controller.initialize