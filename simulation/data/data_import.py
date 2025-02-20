import pandas as pd
import datetime
from typing import Optional, List, Dict

file_path = "data/house_data.csv"

def get_aggregate_timeframe(start: datetime.datetime, stop: datetime.datetime) -> Optional[str]:
    time_difference = stop - start
    if time_difference < datetime.timedelta(minutes=10):
        return "1s"
    elif time_difference < datetime.timedelta(days=1):
        return "10m"
    elif time_difference < datetime.timedelta(days=7):
        return "1h"
    else:
        return None 

def get_data_from_file(file_path: str, measurement: str, field: str, house_id: int, start: datetime.datetime, stop: Optional[datetime.datetime] = None) -> Optional[List[Dict]]:
    df = pd.read_csv(file_path, comment='#')  # Skip metadata lines
    df = pd.DataFrame(df)  # Ensure it is a DataFrame
    df['_time'] = pd.to_datetime(df['_time'])
    
    start_time = start.replace(tzinfo=datetime.timezone.utc)
    stop_time = datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc) if stop is None else stop.replace(tzinfo=datetime.timezone.utc)
    
    timeframe = get_aggregate_timeframe(start_time, stop_time)
    if timeframe is None:
        return None
    
    # Filter data based on conditions
    df = df[(df['_measurement'] == measurement) &
            (df['_field'] == field) &
            (df['house_id'] == str(house_id)) &
            (df['_time'] >= start_time) &
            (df['_time'] <= stop_time)]
    
    # Resampling for aggregation (assuming numeric values, adjust as needed)
    df.set_index('_time', inplace=True)
    
    if df['_value'].dtype == 'object':  # If categorical data
        aggregated = df.resample(timeframe).agg({'_value': lambda x: x.mode()[0] if not x.empty else None})
    else:
        aggregated = df.resample(timeframe).mean()
    
    return aggregated.reset_index().to_dict(orient='records')

# Example usage

print(get_data_from_file(file_path, "heatPumpController", "fanLevel", 1, datetime.datetime.fromisoformat("2024-11-13T11:10:00Z"), datetime.datetime.fromisoformat("2024-11-13T11:20:00Z")))
