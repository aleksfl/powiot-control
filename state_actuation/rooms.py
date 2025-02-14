from device.sensibo import SensiboController
from device.device_type import DeviceType
import simulation.simulation

def get_rooms(type: DeviceType, api_key=None):
    if not isinstance(type, DeviceType):
        raise ValueError(f"Unsupported DeviceType: {type}")    

    match type:
        case DeviceType.SIMULATION:
            return simulation.simulation.rooms
        case DeviceType.SENSIBO:
            return SensiboController.get_instance(api_key).get_available_locations()   
        case _:
            raise ValueError(f"Unsupported DeviceType: {type}")