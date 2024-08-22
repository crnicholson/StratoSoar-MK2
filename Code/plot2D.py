import pandas as pd
import math
import matplotlib.pyplot as plt

# Constants
EARTH_RADIUS = 6371000  # Radius of the Earth in meters


def calculate_flight_path_from_csv(csv_filename, initial_lat, initial_lon, speed):
    # Load the CSV file into a DataFrame using pandas
    df = pd.read_csv(csv_filename)

    # Initialize lists to store calculated latitudes and longitudes
    latitudes = []
    longitudes = []

    # Initialize starting position
    current_lat = initial_lat
    current_lon = initial_lon

    # Iterate over the DataFrame rows using pandas
    for index, row in df.iterrows():
        time = row.iloc[0]
        altitude = row.iloc[5]
        yaw = row.iloc[1]
        pitch = row.iloc[2]

        # Convert degrees to radians
        yaw_rad = math.radians(yaw)
        pitch_rad = math.radians(pitch)

        # Calculate the horizontal and vertical velocity components
        horizontal_speed = speed * math.cos(pitch_rad)

        # Calculate the time step (assuming the data is ordered by time)
        if index == 0:
            time_step = 0
        else:
            time_step = row.iloc[0] - df.iloc[index - 1].iloc[0]

        # Calculate horizontal distance traveled during this time step
        horizontal_distance = horizontal_speed * time_step

        # Calculate the change in latitude and longitude
        delta_lat = horizontal_distance * math.cos(yaw_rad) / EARTH_RADIUS
        delta_lon = (
            horizontal_distance
            * math.sin(yaw_rad)
            / (EARTH_RADIUS * math.cos(math.radians(current_lat)))
        )

        # Update latitude and longitude
        current_lat += math.degrees(delta_lat)
        current_lon += math.degrees(delta_lon)

        # Append to lists
        latitudes.append(current_lat)
        longitudes.append(current_lon)

    return latitudes, longitudes


def plot_flight_path(latitudes, longitudes):
    plt.figure(figsize=(10, 6))
    plt.plot(longitudes, latitudes, marker="o", linestyle="-", color="b")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Glider Flight Path")
    plt.grid(True)
    plt.show()


# Example usage
csv_filename = "descent.csv"  # Replace with the path to your CSV file
initial_latitude = 42.306705  # Initial latitude in degrees
initial_longitude = -71.336548  # Initial longitude in degrees
speed = 30.0  # m/s, assume constant speed

latitudes, longitudes = calculate_flight_path_from_csv(
    csv_filename, initial_latitude, initial_longitude, speed
)

plot_flight_path(latitudes, longitudes)
