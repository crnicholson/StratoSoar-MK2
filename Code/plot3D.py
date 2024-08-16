import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from math import sin, cos

EARTH_RADIUS = 6371000 

data = pd.read_csv("descent.csv")


# Initial conditions.
initial_lat = 42.306705  
initial_lon = -71.336548  
initial_altitude = data["Altitude"][0] 

data["Yaw_rad"] = np.radians(data["Yaw"])

latitudes = [initial_lat]
longitudes = [initial_lon]
altitudes = [initial_altitude]

# Iterate over each row in the DataFrame to calculate the new positions.
for i in range(1, len(data)):
    time_interval = data["Seconds"][i] - data["Seconds"][i - 1] 
    yaw = data["Yaw_rad"][i]
    altitude = data["Altitude"][i] 

    horizontal_distance = 10 * time_interval 

    dlat = horizontal_distance * cos(yaw) / EARTH_RADIUS
    dlon = (
        horizontal_distance * sin(yaw) / (EARTH_RADIUS * cos(np.radians(latitudes[-1])))
    )

    new_lat = latitudes[-1] + np.degrees(dlat)
    new_lon = longitudes[-1] + np.degrees(dlon)

    latitudes.append(new_lat)
    longitudes.append(new_lon)
    altitudes.append(altitude) 

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.plot(latitudes, longitudes, altitudes, marker="o")
ax.set_xlabel("Latitude")
ax.set_ylabel("Longitude")
ax.set_zlabel("Altitude (m)")

plt.title("Glider Flight Path")
plt.show()
