import pandas as pd

GNSS_PATH = 'gnss_data.csv'
METRICS_PATH = 'metrics_data.csv'
TOLERANCE = 0.5
OUTPUT_FILENAME = 'merged_data.csv'

gnss_data = pd.read_csv(GNSS_PATH)
metrics_data = pd.read_csv(METRICS_PATH)

gnss_data['timestamp'] = gnss_data['timestamp'].astype(float)
metrics_data['timestamp'] = metrics_data['timestamp'].astype(float)

gnss_data.sort_values('timestamp', inplace=True)
metrics_data.sort_values('timestamp', inplace=True)

# merge data based on tolerance
merged_data = pd.merge_asof(gnss_data, metrics_data, on='timestamp', tolerance=TOLERANCE, direction='nearest')
merged_data.dropna(inplace=True)

# save to csv
merged_data.to_csv(OUTPUT_FILENAME, index=False)

print(f"Merging complete. Data saved to {OUTPUT_FILENAME}")
