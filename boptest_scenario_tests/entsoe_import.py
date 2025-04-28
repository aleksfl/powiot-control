from entsoe import EntsoePandasClient
from datetime import timedelta
import pandas as pd
import time
import csv

# Setup
api_key = "4333cf49-81c9-4286-8e7e-2047e90f2162"
client = EntsoePandasClient(api_key=api_key)

# ENTSO-E bidding zone for Belgium (Brussels)
area = "BE"

# Define timezone-aware timestamps using pandas
start_date = pd.Timestamp("2024-04-01T00:00:00", tz="Europe/Brussels")
end_date = pd.Timestamp("2024-12-31T23:59:59", tz="Europe/Brussels")

output_file = "brussels_hourly_prices_2024_entsoe.csv"

# Write to CSV
with open(output_file, mode="w", newline="", encoding="utf-8") as file:
    writer = csv.writer(file)
    writer.writerow(["Timestamp", "Price_EUR_per_MWh"])

    current = start_date
    while current <= end_date:
        success = False
        attempt = 1
        while not success:
            try:
                print(f"Fetching hourly prices for {current.date()} (Attempt {attempt})")

                # Query one day at a time
                next_day = current + timedelta(days=1)
                prices = client.query_day_ahead_prices(area, start=current, end=next_day)

                for timestamp, price in prices.items():
                    writer.writerow([timestamp.strftime("%Y-%m-%d %H:%M:%S"), round(price, 2)])

                success = True  # only reached if no exception occurs

            except Exception as e:
                print(f"Failed on {current.date()} (Attempt {attempt}): {e}")
                attempt += 1
                time.sleep(5)  # wait 5 seconds before retrying

        current += timedelta(days=1)

print(f"\nHourly prices saved to: {output_file}")
