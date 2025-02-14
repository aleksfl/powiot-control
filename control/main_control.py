### The purpose of this module is facilitating closed-loop control by providing
#  various control algorithms with inputs and actuation ability connected to external smart home devices.
# ###
import time
from state_actuation import rooms, power, temperature
from state_actuation.state_actuation import Source
from state_actuation.device.device_type import DeviceType
from state_actuation.device import sensibo
from simulation import simulation
import os

system_type = os.getenv("SYSTEM_TYPE")
# Could be different for each device, for now we set all of them to be equal.
datapoint_source = os.getenv("DATAPOINT_SOURCE")

is_simulation = system_type == "SIMULATION"
# Should also be set externally, currently hardcoded for simplicity.
devices = [DeviceType.SENSIBO]
api_keys = {
    device: os.getenv(f"{device.name.upper()}_KEY") for device in devices
}

def control_algorithm(states):
    return black_box_control_algorithm(states)    


def main_control_loop():
    # Control interval in seconds.
    interval = 60
    while True:
        states = []
        if is_simulation:
            states = simulation.get_simulation_states()
        else:
            for device in devices:
                match type:                    
                    case DeviceType.SENSIBO:
                        for r in sensibo.SensiboController.get_available_locations():
                            states.append(temperature.get_current_temperature(device, datapoint_source, room=r, api_key=api_keys[DeviceType.SENSIBO]))                           
                    case _:
                        raise ValueError(f"Unsupported DeviceType: {type}")
                    
        control_algorithm(states)
        time.sleep(interval)



