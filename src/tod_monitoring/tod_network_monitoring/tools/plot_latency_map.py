import pandas as pd
import folium
from scipy.ndimage import gaussian_filter1d
import matplotlib.pyplot as plt
from matplotlib.colors import PowerNorm

INPUT_FILEPATH = 'merged_data.csv'
OUTPUT_FILEPATH = 'latency_map.html'
COLORMAP = 'YlOrRd' # see matplotlib for more colormaps

# load the merged data
df = pd.read_csv(INPUT_FILEPATH)

# apply gaussian smoothing to latitude and longitude
df['latitude'] = gaussian_filter1d(df['latitude'], sigma=0.9)
df['longitude'] = gaussian_filter1d(df['longitude'], sigma=0.9)

# normalization to adjust color mapping based on latency
norm = PowerNorm(gamma=0.95, vmin=df['latency'].min(), vmax=df['latency'].max())

# obtain colormap
cmap = plt.get_cmap(COLORMAP)

def quality_color(latency):
    return plt.cm.colors.to_hex(cmap(norm(latency)))

# initialize CartoDB positron map
map = folium.Map(
    location=[df.iloc[0]['latitude'], df.iloc[0]['longitude']],
    tiles='CartoDB positron',
    zoom_start=15
)

# add points and lines to the map
for index, row in df.iterrows():
    folium.CircleMarker(
        location=[row['latitude'], row['longitude']],
        radius=4,
        color=quality_color(row['latency']),
        fill=True,
        fill_color=quality_color(row['latency']),
        fill_opacity=0.9,
        popup=f"Latency: {row['latency']:.2f}"
    ).add_to(map)

    if index < len(df) - 1:
        next_row = df.iloc[index + 1]
        folium.PolyLine(
            locations=[
                [row['latitude'], row['longitude']],
                [next_row['latitude'], next_row['longitude']]
            ],
            color=quality_color(row['latency']),
            weight=4,
            opacity=0.9
        ).add_to(map)

# save the map to an HTML file
map.save(OUTPUT_FILEPATH)
print(f"map has been saved to {OUTPUT_FILEPATH}")
