import os
import utm
import numpy as np
import pandas as pd


def process_csv(map_path):
    df = pd.read_csv(map_path)

    cones = df[['latitude', 'longitude']].to_numpy()
    if cones.shape[0] == 0:
        print("No cones were recorded!")
        return
    # Convert to UTM
    for i in range(cones.shape[0]):
        utm_coord = utm.from_latlon(cones[i][0], cones[i][1])
        cones[i] = np.array([utm_coord[0], utm_coord[1]])

    car_start = df[df['type'] == 'car_start'][['latitude', 'longitude']].to_numpy()
    if car_start.shape[0] == 0:
        print("No CAR_START, cannot process cones.")
        return
    # Convert to UTM
    utm_coord = utm.from_latlon(car_start[0, 0], car_start[0, 1])
    car_start = np.array([utm_coord[0], utm_coord[1]])

    big_orange = df[df['type'] == 'big_orange'][['latitude', 'longitude']].to_numpy()
    if big_orange.shape[0] < 4:
        print("Less than 4 big_orange, results may be weird")
    # Convert to UTM
    for i in range(big_orange.shape[0]):
        utm_coord = utm.from_latlon(big_orange[i][0], big_orange[i][1])
        big_orange[i] = np.array([utm_coord[0], utm_coord[1]])

    # Find the 4 closest big_orange cones (i.e finish line)
    sorted_oranges = np.argsort(np.sum((big_orange - car_start)**2, axis=1))
    if len(sorted_oranges) > 4:
        sorted_oranges = sorted_oranges[:4]

    # Calculate the average of the 4 big_orange cones
    orange_center = np.mean(big_orange[sorted_oranges], axis=0)

    # Calculate transformation of cones so that car_start is the origin and the
    # center of the finish line is in the x-direction
    orange_center_p = orange_center - car_start
    theta = np.arctan2(orange_center_p[1], orange_center_p[0])
    R = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])
    new_cones = (cones - car_start) @ R.T
    df[['latitude', 'longitude']] = new_cones

    # Ensure car_start is zero-ed out
    car_start_index = df.index[df['type'] == 'car_start'].to_list()
    print(car_start_index)
    df.loc[car_start_index[0]] = {
        'type': 'car_start',
        'latitude': 0,
        'longitude': 0,
        'latitude_var': 0,
        'longitude_var': 0,
        'covariance': 0
    }

    # Convert to eufs_sim format
    df.columns = ['tag', 'x', 'y', 'x_variance', 'y_variance', 'xy_covariance']
    df.insert(loc=3, column='direction', value=np.zeros(new_cones.shape[0]))

    # Save in a new file
    df.to_csv(os.path.splitext(map_path)[0] + "_xy.csv", index=False)
