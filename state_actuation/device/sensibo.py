import requests
import json

_SERVER = "https://home.sensibo.com/api/v2"


class SensiboClientAPI(object):
    def __init__(self, api_key):
        self._api_key = api_key

    def _get(self, path, **params):
        params["apiKey"] = self._api_key
        response = requests.get(_SERVER + path, params=params, timeout=15)
        response.raise_for_status()
        return response.json()

    def _patch(self, path, data, **params):
        params["apiKey"] = self._api_key
        response = requests.patch(_SERVER + path, params=params, data=data, timeout=15)
        response.raise_for_status()
        return response.json()

    def devices(self):
        result = self._get("/users/me/pods", fields="id,room")
        return {x["room"]["name"]: x["id"] for x in result["result"]}

    def pod_measurement(self, podUid):
        result = self._get("/pods/%s/measurements" % podUid)
        return result["result"]

    def pod_ac_state(self, podUid):
        result = self._get(
            "/pods/%s/acStates" % podUid, limit=1, fields="status,reason,acState"
        )
        return result["result"][0]["acState"]

    def pod_change_ac_state(self, podUid, currentAcState, propertyToChange, newValue):
        self._patch(
            "/pods/%s/acStates/%s" % (podUid, propertyToChange),
            json.dumps({"currentAcState": currentAcState, "newValue": newValue}),
        )

class SensiboController:
    _instance = None  # Static instance storage

    def __init__(self, api_key: str):
        if not api_key:
            raise ValueError("API key must be provided")
        self._api_key = api_key
        self._sensibo_client = SensiboClientAPI(api_key)
        self._devices = self._sensibo_client.devices()

    @classmethod
    def get_instance(cls, api_key: str):
        """Get the instance, initializing it if necessary."""
        if cls._instance is None:
            cls._instance = cls(api_key)
        return cls._instance

    def get_latest_measurement(self, location: str):
        uid = self._devices.get(location)
        return self._sensibo_client.pod_measurements(uid)

    def get_available_locations(self):
        return list(self._devices.keys())

    def set_temperature(self, location: str, temperature: float) -> bool:
        if location not in self.get_available_locations():
            raise ValueError("Location not found", location)

        uid = self._devices.get(location)
        ac_state = self._sensibo_client.pod_ac_state(uid)
        self._sensibo_client.pod_change_ac_state(
            self._devices[location], ac_state, "targetTemperature", temperature
        )
        return True