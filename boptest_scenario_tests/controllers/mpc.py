"""
Custom MPC controller for managing the temperature of the hydronic heat pump system.
"""

import casadi as ca
import numpy as np

class MPCController:
    def __init__(self, setpoint=273.15+22, dt=300, N = 10):  
        self.setpoint = setpoint              
        self.dt = dt
        self.N = N    
        
        self.opti = ca.Opti()                
        self.X = self.opti.variable(self.N+1)
        self.U = self.opti.variable(self.N)                
        self.x_ref = self.opti.parameter()  # Reference state (temperature setpoint)

        # Define system dynamics for a nonlinear system
        # Typical MPC boilerplate dynamics.
        #for k in range(self.N):
        #    # Nonlinear dynamics: x_{k+1} = a*x_k + b*u_k + c*u_k^2
        #    self.opti.subject_to(self.X[k+1] == self.a * self.X[k] + self.b * self.U[k] + self.c * self.U[k]**2)

        # Define the cost function (tracking error + control effort)
        # Should involve power consumption as well
        cost = ca.sumsqr(self.X - self.x_ref) + 0.1 * ca.sumsqr(self.U)
        self.opti.minimize(cost)

        # Control and state constraints
        self.opti.subject_to(self.opti.bounded(0, self.U, 1))
                
        self.opti.solver('ipopt')

    def compute_control(self, y):
        """
        Compute the control action using the current temperature 'y' and optimization.
        """
                
        self.opti.set_value(self.X[0], y['reaTZon_y'])
        self.opti.set_value(self.x_ref, self.setpoint)
                
        sol = self.opti.solve()                
        u_value = sol.value(self.U[0])
                
        u = {
            'oveHeaPumY_u': max(0, min(1, u_value)),  
        }
        return u, self.setpoint - y['reaTZon_y']  

    def initialize(self):        
        print("Initializing MPC controller:")        
        print("Temperature setpoint (*C):", self.setpoint-273.15)                
        print("Prediction horizon:", self.N)
        u = {        
            'oveHeaPumY_activate': 1,        
            'oveTSet_u': self.setpoint,     
        }
        return u
