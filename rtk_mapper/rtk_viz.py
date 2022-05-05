import os
import utm
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node


class RTKViz:
    def __init__(self, path, save, logger):
        # Save parameters
        self.map_path = path
        self.save = save
        self.logger = logger

        # Read file
        assert os.path.isfile(self.map_path), "Map path is not a file"

    def run(self, use_utm=False):
        df = pd.read_csv(self.map_path)

        # Check format of csv file
        fmt1 = ['type', 'latitude', 'longitude', 'latitude_var', 'longitude_var', 'covariance']
        fmt2 = ['tag', 'x', 'y', 'direction', 'x_variance', 'y_variance', 'xy_covariance']
        if list(df.columns) == fmt1:
            if use_utm:
                cones = df[['latitude', 'longitude']].to_numpy()
                # Convert to UTM
                for i in range(cones.shape[0]):
                    utm_coord = utm.from_latlon(cones[i][0], cones[i][1])
                    cones[i] = np.array([utm_coord[0], utm_coord[1]])
                df[['latitude', 'longitude']] = cones

            # Extract data
            cols = ["latitude", "longitude"]
            df_blue = df[df['type'] == 'blue'][cols]
            df_yellow = df[df['type'] == 'yellow'][cols]
            df_orange = df[df['type'].isin(['big_orange', 'orange'])][cols]

            # Create figure
            fig, axs = plt.subplots()
            axs.axis('equal')

            # Plot
            axs = df_blue.plot.scatter("longitude", "latitude", 49,
                                       "#0000ff", edgecolors='k', ax=axs)
            axs = df_yellow.plot.scatter("longitude", "latitude",
                                         49, "#ffff00", edgecolors='k', ax=axs)
            axs = df_orange.plot.scatter("longitude", "latitude",
                                         49, "#ff8c00", edgecolors='k', ax=axs)

        elif list(df.columns) == fmt2:
            # Extract data
            cols = ["x", "y"]
            df_blue = df[df['tag'] == 'blue'][cols]
            df_yellow = df[df['tag'] == 'yellow'][cols]
            df_orange = df[df['tag'].isin(['big_orange', 'orange'])][cols]

            # Create figure
            fig, axs = plt.subplots()
            axs.axis('equal')

            # Plot
            axs = df_blue.plot.scatter("x", "y", 49, "#0000ff", edgecolors='k', ax=axs)
            axs = df_yellow.plot.scatter("x", "y", 49, "#ecef1f", edgecolors='k', ax=axs)
            axs = df_orange.plot.scatter("x", "y", 49, "#ff8c00", edgecolors='k', ax=axs)
        else:
            raise ValueError("CSV file has invalid format")

        axs.set_ylabel("")
        axs.set_xlabel("")

        # Save figure
        if (self.save):
            image_path = os.path.splitext(self.map_path)[0] + ".png"
            plt.tight_layout()
            plt.savefig(image_path)
            self.logger.info(f"Saved figure to {image_path}")

        return plt.gcf(), axs


class RTKVizNode(Node):
    def __init__(self, name="rtk_viz"):
        super().__init__(name)

        EUFS_MASTER = os.environ["EUFS_MASTER"]
        save = self.declare_parameter("save", True).value
        use_utm = self.declare_parameter("use_utm", False).value
        map_path = self.declare_parameter("path", "").value
        assert map_path != "", "Map path is not given"
        if not os.path.isabs(map_path):
            map_path = os.path.join(EUFS_MASTER, map_path)
        self.get_logger().info(f"Reading csv: {map_path}")
        assert os.path.isfile(map_path), "Map path is not a file"

        viz = RTKViz(map_path, save, self.get_logger())
        fig, axes = viz.run(use_utm)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    _ = RTKVizNode()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
