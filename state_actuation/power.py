### Involves getting power datapoints (consumption and cost) from supported devices ###

from simulation.simulation import get_power
from state_actuation.state_actuation import Source
from device.device_type import DeviceType

def get_current_power_consumption(type: DeviceType, source: Source, house_id , api_key = None):    
    if not isinstance(type, DeviceType):
        raise ValueError(f"Unsupported DeviceType: {type}")

    if not isinstance(source, Source):
        raise ValueError(f"Unsupported Source: {source}")

    if source not in Source:
        raise ValueError(f"Unsupported source: {source}")

    match type:
        case DeviceType.SIMULATION:            
            return get_power()
        case DeviceType.TIBBER:
            pass  # Placeholder for TIBBER logic        
        case _:
            raise ValueError(f"Unsupported DeviceType: {type}")


def get_spotprice():
    pass 