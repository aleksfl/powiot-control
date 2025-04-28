"""
Custom MPC controller for managing the temperature of the hydronic heat pump system.
"""

import pandas as pd
from datetime import datetime, timedelta
import sys
import os
hvac_mpc_path = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', 'leap-c-house', 'leap_c', 'examples', 'hvac')
)
sys.path.insert(0, hvac_mpc_path)

import leap_c.examples.hvac.mpc as hvac_mpc
import numpy as np
import matplotlib.pyplot as plt

# Read historical hourly prices from entsoe data.
prices = pd.read_csv("brussels_hourly_prices_2024.csv", parse_dates=["Timestamp"])
prices.set_index("Timestamp", inplace=True)

def get_prices(start_time: datetime, N: int, dt: int) -> np.ndarray:
    print("prices start_time", start_time)
    result = []
    today_date = start_time.date()
    tomorrow_date = today_date + timedelta(days=1)    
    for i in range(N):
        current_time = start_time + timedelta(seconds=i * dt)        
        # Determine which day the current step falls on
        if current_time.date() == today_date:  # Prices for all of today are always available
            day_prices = prices[prices.index.date == today_date]
            known_prices = day_prices[day_prices.index <= current_time]            
            price = known_prices.iloc[-1]["Price_EUR_per_MWh"]            
        elif current_time.date() == tomorrow_date:  # Now is tomorrow, new prices are availble.
            day_prices = prices[prices.index.date == tomorrow_date]
            known_prices = day_prices[day_prices.index <= current_time]            
            price = known_prices.iloc[-1]["Price_EUR_per_MWh"]                        
        else:  # Assume price stays the same if we don't have any information.
            known_prices = prices[prices.index.date == tomorrow_date]            
            price = known_prices.iloc[-1]["Price_EUR_per_MWh"]
        result.append(price)
    return np.array(result)

class MPCController:
    def __init__(self, dt=300, N = 12*24*2):               
        self.dt = dt
        self.N = N           
        self.mpc = None               
        self.ocp_solver = None
        self.kalman_filter = None
        self.filter_initialized = False
        # Represents input signal from MPC in watts.        
        self.last_u = None # Could initialize this to 0 watts, but None achieves the same effect.

    def compute_control(self, y, time: datetime, forecasts=None):
        """Compute the control input from the measurement.
        Parameters
        ----------
        y : dict
            Contains the current values of the measurements.
            {<measurement_name>:<measurement_value>}
        forecasts : structure depends on controller
            Forecasts used to calculate control, defined in ``update_forecasts``..

        Returns
        -------
        u : dict
            Defines the control input to be used for the next step.
            {<input_name> : <input_value>}

        """
                        
        indoor_temp = y['reaTRoo_y']                           

        # Check forecasts and extract set point information                
        try:                        
            #sp_lower = forecasts['LowerSetp[1]']
            #sp_upper = forecasts['UpperSetp[1]']
            temp_forecasts = forecasts['TDryBul']
            solar_forecasts = forecasts['HGloHor']             
        except KeyError:
            raise KeyError("Forecast values ['LowerSetp[1]', 'UpperSetp[1]', 'TDryBul', 'HGloHor'] not in forecasts: {0}".format(forecasts.columns))
        
        # The kalman filter is initialized here so the initial temperature measurement can be used for the initial states.
        if not self.filter_initialized:            
            Q = np.diag([0.01, 0.01, 0.01])  # Process noise
            R = np.array([[0.5]])            # Measurement noise            
            x0 = np.matrix([indoor_temp, indoor_temp, indoor_temp]).T # Assume radiator is off to start with and there exists thermal equilibrium
            P = np.eye(3) * 1.0               # Initial uncertainty            
            self.kalman_filter = hvac_mpc.create_kalman_filter(self.mpc.ocp, Q, R, x0, P)
            self.filter_initialized = True


        # Run Kalman filter
        self.kalman_filter.predict(u=self.last_u)
        self.kalman_filter.update(np.array([[indoor_temp]]))
        print("Current indoor temp", indoor_temp - 273.15)
        print("Current outdoor temp", temp_forecasts[0] - 273.15)
        # Set initial state for MPC 
        print("Kalman_filter:", self.kalman_filter.x)          
        # ocp solver contstraint set             
        self.ocp_solver.set(0, "x", self.kalman_filter.x)                           

        prices = get_prices(time, self.N, self.dt)        
        parameter_values = np.vstack([np.array(temp_forecasts), np.array(solar_forecasts), prices]).T        

        for stage in range(self.N):
            self.ocp_solver.set(stage, "p", parameter_values[stage, :])        
             
        # Set boundaries based on setpoints.
        #self.ocp_solver.set(stage, "lbx", sp_lower)       
        #self.ocp_solver.set(stage, "ubx", sp_upper)                

        # Solve MPC
        status = self.ocp_solver.solve_for_x0(self.kalman_filter.x)

        if status != 0:
            raise RuntimeError(f"[MPC] Solver failed with status code {status}")                        

        qh = self.ocp_solver.get(0, "u")[0]
        print("MPC output", qh)        
        self.last_u = np.array([[qh]])  # Save for next filter step        
        # Convert from power input to setpoint temperature and binary activation signal.

        enable_signal = 0
        supply_temperature_setpoint = 293.15

        qh_nom = 5000 # 5kW from documentation
        delta_T_nom = 60 # Nominal power range 20-80 C

        # Ignore signals under 1 watt and turn off the heater.
        if qh >= 1:
            enable_signal = 1
            supply_temperature_setpoint = np.clip(indoor_temp + delta_T_nom * (qh / qh_nom) ** (1 / 1.3), 293.15, 353.15)

        print("Simulation inputs: ")
        print("oveTSetSup_u", supply_temperature_setpoint)
        print("ovePum_u", enable_signal)
        u = {
            'oveTSetSup_u': supply_temperature_setpoint,
            'ovePum_u': enable_signal,      
        }

        return u

    def initialize(self):        
        print("Initializing MPC controller")        
        self.mpc = hvac_mpc.HvacMpc(N_horizon=self.N, T_horizon=self.dt*self.N)                
        self.ocp_solver = hvac_mpc.AcadosOcpSolver(acados_ocp=self.mpc.ocp)                    

        u = {        
            'oveTSetSup_activate': 1,   
            'ovePum_u': 0,                    
        }    
        return u       
    
    def get_forecast_parameters(self):
        """Get forecast parameters within the controller.

        Returns
        -------
        forecast_parameters: dict
            {'point_names':[<string>],
            'horizon': <int>,
            'interval': <int>}

        """

        forecast_parameters = {'point_names':['LowerSetp[1]', # Lower temperature set point for thermal comfort of zone
                                            'UpperSetp[1]', # Upper temperature set point for thermal comfort of zone
                                            'TDryBul', # Outside temperature forecast
                                            'HGloHor'], # Global horizontal solar radiation forecast
                            'horizon': (self.N * self.dt) - self.dt,
                            'interval': self.dt}


        return forecast_parameters
        