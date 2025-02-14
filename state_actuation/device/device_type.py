from enum import Enum

# Currently supported device types, note that this also included simulated values and external APIs.
class DeviceType(Enum):
    SIMULATION = 1
    SENSIBO = 2
    TIBBER = 3
    MET = 4
    SOLCAST = 4