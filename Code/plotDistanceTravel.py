# To-do
# - Add KML output
# - Add a way to input the final coordinates
# - Add a way to calculate speed based on final coordunates


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos, radians, atan2, sqrt, degrees
 
EARTH_RADIUS = 6371000 

data = pd.read_csv("descent.csv")

# Initial conditions.
initial_lat = 42.306705
initial_lon = -71.336548
initial_altitude = data["Altitude"][0]
real_seconds = 3.25

data["Yaw_rad"] = np.radians(data["Yaw"])

latitudes = [initial_lat]
longitudes = [initial_lon]
altitudes = [initial_altitude]

total_ground_distance = 0
total_3d_distance = 0

for i in range(1, len(data)):
    time_interval = (
        data["Seconds"][i] * real_seconds - data["Seconds"][i - 1] * real_seconds
    )
    yaw = data["Yaw_rad"][i]
    altitude = data["Altitude"][i]

    horizontal_distance = 10 * time_interval

    dlat = horizontal_distance * cos(yaw) / EARTH_RADIUS
    dlon = horizontal_distance * sin(yaw) / (EARTH_RADIUS * cos(radians(latitudes[-1])))

    new_lat = latitudes[-1] + degrees(dlat)
    new_lon = longitudes[-1] + degrees(dlon)

    # Calculate ground distance using the Haversine formula.
    lat1, lon1 = radians(latitudes[-1]), radians(longitudes[-1])
    lat2, lon2 = radians(new_lat), radians(new_lon)

    dlat_haversine = lat2 - lat1
    dlon_haversine = lon2 - lon1

    a = (
        sin(dlat_haversine / 2) ** 2
        + cos(lat1) * cos(lat2) * sin(dlon_haversine / 2) ** 2
    )
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    ground_distance = EARTH_RADIUS * c
    total_ground_distance += ground_distance

    # Calculate 3D distance.
    altitude_diff = altitude - altitudes[-1]
    distance_3d = sqrt(ground_distance**2 + altitude_diff**2)
    total_3d_distance += distance_3d

    latitudes.append(new_lat)
    longitudes.append(new_lon)
    altitudes.append(altitude)


# Smooth the data for plotting.
def moving_average(data, window_size):
    return np.convolve(data, np.ones(window_size) / window_size, mode="valid")


window_size = 10
latitudes_smooth = moving_average(latitudes, window_size)
longitudes_smooth = moving_average(longitudes, window_size)
altitudes_smooth = moving_average(altitudes, window_size)

min_altitude_index = np.argmin(altitudes_smooth)

min_altitude = altitudes_smooth[min_altitude_index]
min_latitude = latitudes_smooth[min_altitude_index]
min_longitude = longitudes_smooth[min_altitude_index]

print(f"End spot altitude: {min_altitude} meters")
print(f"Coordinates of end spot: {min_latitude}, {min_longitude}")
print(f"Total Ground Distance: {total_ground_distance:.2f} meters")
print(f"Total 3D Distance: {total_3d_distance:.2f} meters")

# Plot the 3D flight path.
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.plot(latitudes_smooth, longitudes_smooth, altitudes_smooth, label="Flight Path")
ax.scatter(
    latitudes_smooth[0],
    longitudes_smooth[0],
    altitudes_smooth[0],
    color="green",
    s=100,
    label="Start Point",
)
ax.scatter(
    latitudes_smooth[-1],
    longitudes_smooth[-1],
    altitudes_smooth[-1],
    color="red",
    s=100,
    label="End Point",
)

ax.set_xlabel("Latitude")
ax.set_ylabel("Longitude")
ax.set_zlabel("Altitude (m)")

plt.title("Glider Flight Path")
plt.legend()
plt.show()
