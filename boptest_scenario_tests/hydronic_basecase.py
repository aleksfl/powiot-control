# -*- coding: utf-8 -*-
"""
This script demonstrates the basecase of the bestest_hydronic_heat_pump scenario.
Uses the baseline controller, which provides no control inputs to the simulation
It uses the testing interface implemented in interface.py and the concrete controller implemented
in controllers/pid.py.
"""

# GENERAL PACKAGE IMPORT
# ----------------------
import sys
import os
import matplotlib
matplotlib.use("Agg")  # Use non-GUI backend before importing pyplot to be able to run in WSL
from matplotlib import pyplot as plt
import numpy as np
sys.path.insert(0, '/'.join((os.path.dirname(os.path.abspath(__file__))).split('/')[:-2]))
from interface import control_test


def run(plot=True, length=3600, step=300, start_time=0, warmup_period=0):
    """Run test case.
    Parameters
    ----------
    plot : bool, optional
        True to plot timeseries results.
        Default is False.
    length : int, optional
        Desired simulation length in seconds.
        Default is 3600 (1 hour).
    step : int, optional
        Desired simulation step size.
        Default is 300 (5 minutes)
    start_time : int, optional
        Desired simulation start time.
        Default is 0.
    warmup_period : int, optional
        Desired simulation warmup period before start time.
        Default is 0.

    Returns
    -------
    kpi : dict
        Dictionary of core KPI names and values.
        {kpi_name : value}
    res : dict
        Dictionary of trajectories of inputs and outputs.
    custom_kpi_result: dict
        Dictionary of tracked custom KPI calculations.
        Empty if no customized KPI calculations defined.

    """

    # RUN THE CONTROL TEST
    # --------------------
    control_module = 'controllers.baseline'
    scenario = {'time_period': 'typical_heat_day', 'electricity_price': 'highly_dynamic'}
    start_time = 0
    warmup_period = 0
    length = length
    step = step
    # ---------------------------------------

    # RUN THE CONTROL TEST
    # --------------------
    kpi, df_res, custom_kpi_result, forecasts = control_test('bestest_hydronic_heat_pump',
                                                            control_module,
                                                             scenario=scenario,
                                                           start_time=start_time,
                                                             warmup_period=warmup_period,
                                                             length=length,
                                                             step=step)

    # POST-PROCESS RESULTS
    # --------------------
    time = df_res.index.values / 3600  # convert s --> hr
    zone_temperature = df_res['reaTZon_y'].values - 273.15  # convert K --> C    

    if plot:                
        plt.figure(1)
        plt.title('Zone Temperature')
        plt.plot(time, zone_temperature, label="Zone Temperature")
        plt.plot(time, 20 * np.ones(len(time)), '--', label="Lower Comfort Limit (20°C)")
        plt.plot(time, 25 * np.ones(len(time)), '--', label="Upper Comfort Limit (25°C)")
        plt.ylabel('Temperature [C]')
        plt.xlabel('Time [hr]')
        plt.legend()
        # Save the plot as an image instead of showing it
        plt.savefig("temperature_basecase.png", dpi=300, bbox_inches='tight')
        print("Plot saved as 'temperature_basecase.png'")        

    return kpi, df_res, custom_kpi_result


if __name__ == "__main__":
    kpi, df_res, custom_kpi_result = run()
