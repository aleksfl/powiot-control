"""
PI controller for managing the temperature of the hydronic heat pump system.
"""

class PIController:
    def __init__(self, k_p=1000, k_i=1, setpoint=273.15+22, dt=300):        
        self.k_p = k_p
        self.k_i = k_i
        self.setpoint = setpoint
        self.dt = dt
        self.integral = 0  # Integral term for the controller

    def compute_control(self, y, forecasts=None):        
        error = self.setpoint - y['reaTZon_y']
        self.integral += error * self.dt

        u_value = self.k_p * error + self.k_i * self.integral
        # Anti-windup mechanism
        if u_value <= 0 or u_value >= 1:
            self.integral -= error * self.dt
        u = {
            'oveHeaPumY_u': max(0, min(1, u_value)),        
        }
        return u, error

    def initialize(self):        
        print("Initializing PI controller:")
        print("K_p:", self.k_p)
        print("K_i:", self.k_i)
        print("Temperature setpoint (*C):", self.setpoint-273.15)
        u = {        
            'oveHeaPumY_activate': 1,        
            'oveTSet_u': self.setpoint,     
        }
        return u