### Involves getting and setting temperature values for supported devices ###

import custom_simulation.simulation
from example_control.state_actuation import Source
from device.sensibo import SensiboController
from device.device_type import DeviceType
from datetime import datetime

def get_current_temperature(type: DeviceType, source: Source, house_id=None, room=None, api_key=None):
    if not isinstance(type, DeviceType):
        raise ValueError(f"Unsupported DeviceType: {type}")
    
    if not isinstance(source, Source):
        raise ValueError(f"Unsupported Source: {source}")

    match type:
        case DeviceType.SIMULATION:
            return custom_simulation.simulation.get_current_temperature()
        case DeviceType.SENSIBO:
            return SensiboController.get_instance(api_key).get_latest_measurement(room)   
        case _:
            raise ValueError(f"Unsupported DeviceType: {type}")

def get_historical_temperatures(type: DeviceType, source: Source, from_time: datetime, house_id=None, room=None, api_key=None):
    if not isinstance(type, DeviceType):
        raise ValueError(f"Unsupported DeviceType: {type}")
    
    if not isinstance(source, Source):
        raise ValueError(f"Unsupported Source: {source}")

    match type:
        case DeviceType.SIMULATION:
            return custom_simulation.simulation.get_historical_temperatures(from_time)
        case DeviceType.SENSIBO:
            pass        
        case _:
            raise ValueError(f"Unsupported DeviceType: {type}")

def set_temperature(type: DeviceType, source: Source, temperature: float, house_id=None, room=None, api_key=None):
    if not isinstance(type, DeviceType):
        raise ValueError(f"Unsupported DeviceType: {type}")
    
    if not isinstance(source, Source):
        raise ValueError(f"Unsupported Source: {source}")

    match type:
        case DeviceType.SIMULATION:
            return custom_simulation.simulation.set_temperature(temperature)
        case DeviceType.SENSIBO:
            return SensiboController.get_instance(api_key).set_temperature(room, temperature)
        case _:
            raise ValueError(f"Unsupported DeviceType: {type}")

def get_outside_temperature(type: DeviceType, source: Source, house_id=None, room=None, api_key=None):
    if not isinstance(type, DeviceType):
        raise ValueError(f"Unsupported DeviceType: {type}")
    
    if not isinstance(source, Source):
        raise ValueError(f"Unsupported Source: {source}")

    match type:
        case DeviceType.SIMULATION:
            return custom_simulation.simulation.get_outside_temperature()
        case DeviceType.MET:
            pass
        case DeviceType.SOLCAST:
            pass
        case _:
            raise ValueError(f"Unsupported DeviceType: {type}")
        
def get_historic_outside_temperatures(type: DeviceType, source: Source, from_time: datetime, house_id=None, api_key=None):
    if not isinstance(type, DeviceType):
        raise ValueError(f"Unsupported DeviceType: {type}")
    
    if not isinstance(source, Source):
        raise ValueError(f"Unsupported Source: {source}")

    match type:
        case DeviceType.SIMULATION:
            return custom_simulation.simulation.get_historical_outside_temperatures()
        case DeviceType.MET:
            pass
        case DeviceType.SOLCAST:
            pass
        case _:
            raise ValueError(f"Unsupported DeviceType: {type}")