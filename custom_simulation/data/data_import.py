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
    print(f"Reading data from file: {file_path}")
    dtype_mapping = {
        "_field": str,
        "_measurement": str,
        "house_id": str,
        "_value": str  # Assuming mixed types; adjust if necessary
    }                
    
    # Read CSV using assigned header
    df = pd.read_csv(file_path, dtype=dtype_mapping, low_memory=False)
    
    print("Columns in dataframe:", df.columns)  
    
    if '_time' not in df.columns:
        raise ValueError("Column '_time' not found in CSV file. Check the header row.")
    
    df['_time'] = pd.to_datetime(df['_time'], errors='coerce')  # Handle invalid timestamps gracefully
    
    # Drop rows where _time could not be converted
    df = df.dropna(subset=['_time'])
    
    print(f"Total rows after dropping invalid timestamps: {len(df)}")
    
    start_time = start.replace(tzinfo=datetime.timezone.utc)
    stop_time = datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc) if stop is None else stop.replace(tzinfo=datetime.timezone.utc)
    
    timeframe = get_aggregate_timeframe(start_time, stop_time)
    print(f"Using aggregation timeframe: {timeframe}")
    
    if timeframe is None:
        return None
    
    # Filter data based on conditions
    df = df[(df['_measurement'] == measurement) &
            (df['_field'] == field) &
            (df['house_id'] == str(house_id)) &
            (df['_time'] >= start_time) &
            (df['_time'] <= stop_time)]
    
    print(f"Total rows after filtering: {len(df)}")
    
    # Resampling for aggregation (assuming numeric values, adjust as needed)
    df.set_index('_time', inplace=True)
    
    if df['_value'].dtype == 'object':  # If categorical data
        aggregated = df.resample(timeframe).agg({'_value': lambda x: x.mode()[0] if not x.empty else None})
    else:
        df['_value'] = pd.to_numeric(df['_value'], errors='coerce')  # Convert _value to numeric if possible
        aggregated = df.resample(timeframe).mean()
    
    print(f"Total rows after aggregation: {len(aggregated)}")
    
    return aggregated.reset_index().to_dict(orient='records')
# Example usage
#print(get_data_from_file(file_path, "heatPumpController", "fanLevel", 1, datetime.datetime.fromisoformat("2024-11-13T11:10:00Z"), datetime.datetime.fromisoformat("2024-11-13T11:20:00Z")))
