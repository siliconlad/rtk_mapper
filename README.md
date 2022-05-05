# rtk_mapper

This package makes it easy to record positions of cones using an rtk-based receiver publishing to the `/fix` topic.

## Topic

The `rtk_mapper` node subscribes to the `/fix` topic. It must have type [`sensor_msgs/msg/NavSatFix`](http://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html).

## Parameters

The `rtk_mapper` has the following parameters:

1. `debug`: enable debug logging
2. `rtk_map`: path to a csv file whose contents are copied into the new csv file
3. `threshold`: covariance threshold for measurement

## Usage

Note, you must have the environment variable `$EUFS_MASTER` pointing to your workspace and the directory `$EUFS_MASTER/map` must exist.

To use, follow these steps:

1. Select a color for the cone (`b` for `BLUE`, `y` for `YELLOW`, `o` for `ORANGE`, `s` for `BIG_ORANGE`). Note: whichever color you select will persist until you change it by pressing one of the other 3 keys.
2. Everytime you hold the spacebar down, the node will buffer all incoming messages to the `/fix` topic. Once you release the space bar, it will take the average of latitude and longitude and write this in a csv file in $EUFS_MASTER/map.
3. Selecting `u` to cause the program to enter the UPDATE MODE, where any measured cones will be compared to the existing set of cones in the csv file and replace the closest one.
4. Repeat step 1 and 2 as many times as necessary
5. Press `q` to quit and shutdown the node

A summary of the keyboard shortcuts are below:

1. `b`: choose BLUE color
2. `y`: choose YELLOW color
3. `o`: choose ORANGE color
4. `s`: choose BIG_ORAGE color
5. `d`: delete last measurement
6. `u`: enter UPDATE MODE
7. `q`: quit
8. `<space>`: capture messages

### CSV Format

The csv file has a very simple format:

1. `longitude`: longitude of cone
2. `latitude`: latitude of cone
3. `longitude_var`: longitude variance
4. `latitude_var`: latitude variance
5. `color`: color of cone
