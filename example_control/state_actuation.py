"""Module for aquiring state values and sending actuation signals for control purposes.
 In the smart home context the main ones are temperature (inside rooms and outsiden) and electricity prices """

from enum import Enum

# Determines sources from which state datapoints are acquired, can be different for each device, not relevant for the simulation.
class Source(Enum):
    API = 1
    INFLUXDB = 2
    

# Determines how actuation signals are handled, can be different for each device.
class ActuationType(Enum):
    SIMULATION = 1
    API = 2
    
