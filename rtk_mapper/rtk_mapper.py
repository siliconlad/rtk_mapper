import os
import sys
import signal
import pygame
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from os import path
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import NavSatFix, NavSatStatus

from rtk_mapper import RTKType
from rtk_mapper.rtk_viz import RTKViz
from rtk_mapper.rtk_transformer import process_csv

matplotlib.use('module://rtk_mapper.pygame_matplotlib.backend_pygame')
# longitude: x
# latitude: y


class RTKMapper(Node):
    def __init__(self, name="rtk_mapper"):
        super().__init__(name)

        # Initialise pygame
        pygame.init()
        self.screen = pygame.display.set_mode((700, 700))
        pygame.display.set_caption("keyboard")

        # State
        self.log = False
        self.type = None
        self.update_mode = False
        self.car_start_count = 0
        self.fix_buff = np.array([]).reshape(0, 2)
        self.cov_buff = np.array([]).reshape(0, 2, 2)
        self.EUFS_MASTER = os.environ["EUFS_MASTER"]
        self.thresh = self.declare_parameter("threshold", 0.0002).value
        self.get_logger().info(f"Threshold is {self.thresh}")

        # ROS Parameters
        if self.declare_parameter("debug", False).value:
            self.get_logger().set_level(LoggingSeverity.DEBUG)
            self.get_logger().info("debug logging enabled")

        # Create subscriber
        self.sub_ = self.create_subscription(NavSatFix, "/fix", self.fix_callback, 1)

        # Copy over from given file
        self.rtk_source_path = self.declare_parameter("rtk_map", "").value
        if self.rtk_source_path != "":
            self.get_logger().info("Loading a copy of an existing map")
            # If path is not absolute, we attach the EUFS_MASTER path as prefix
            if not os.path.isabs(self.rtk_source_path):
                self.rtk_source_path = os.path.join(self.EUFS_MASTER, self.rtk_source_path)
            self.get_logger().info(f"Reading csv: {self.rtk_source_path}")
            assert os.path.isfile(self.rtk_source_path), "Map path is not a file"
            with open(self.rtk_source_path, "r") as f:
                f.readline()  # Skips csv header
                rtk_source_map = f.read()

        # Create file to log cones
        assert path.isdir(path.join(self.EUFS_MASTER, "map")), "No map directory in eufs-master"
        date = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
        self.map_path = path.join(self.EUFS_MASTER, "map", f"rtk_map_{date}.csv")
        with open(self.map_path, "x") as f:
            f.write("type,latitude,longitude,latitude_var,longitude_var,covariance\n")
            if self.rtk_source_path != "":
                f.write(rtk_source_map)
        self.get_logger().info("Target csv file initialized")

        # Setup viz
        self.viz = RTKViz(self.map_path, False, self.get_logger)
        self.update()

        # Setup keyboard loop @ 100Hz
        self.timer_ = self.create_timer(0.01, self.loop)

        ###
        self.get_logger().info("--------")
        self.get_logger().info("rtk_mapper is ready!")
        self.get_logger().info("Press b for BLUE, y for YELLOW, o for ORANGE, s for BIG_ORANGE")
        self.get_logger().info("Press u to enter UPDATE MODE")
        self.get_logger().info("Press c to enter CAR START position")
        self.get_logger().info("Press press space to record messages")
        self.get_logger().info("Press press q to quit")

    def loop(self):
        # Handle keyboard events
        for event in pygame.event.get():
            # Logic for when cetain keys are pressed
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.get_logger().info("Shutting down")
                    process_csv(self.map_path)
                    pygame.quit()
                    signal.raise_signal(signal.SIGINT)
                    sys.exit()
                elif event.key == pygame.K_SPACE:
                    # Only accumulate if we've selected a colour
                    have_car_start = self.type == RTKType.CAR_START and self.car_start_count > 0
                    if have_car_start and not self.update_mode:
                        self.get_logger().warn("Already recorded car_start")
                    elif self.type is not None:
                        self.get_logger().info("Start acummulation of messages")
                        self.log = True
                    else:
                        self.get_logger().warn("Please select a cone color first!")
                # As long as we are not accumulating
                elif not self.log:
                    # The `u` key triggers the update mode to update position of a cone
                    if event.key == pygame.K_u:
                        if not self.update_mode:
                            self.get_logger().info("Entering UPDATE MODE")
                            self.update_mode = True
                            if self.rtk_source_path == "":
                                self.get_logger().warn(
                                    "You will lose cone position if you continue.")
                        else:
                            self.get_logger().info("Leaving UPDATE MODE")
                            self.update_mode = False
                    # Only change to blue if not already blue
                    elif event.key == pygame.K_b and self.type != RTKType.BLUE:
                        self.type = RTKType.BLUE
                        self.get_logger().info("Ready to record BLUE cones")
                    # Only change to yellow if not already yellow
                    elif event.key == pygame.K_y and self.type != RTKType.YELLOW:
                        self.type = RTKType.YELLOW
                        self.get_logger().info("Ready to record YELLOW cones")
                    # Only change to orange if not already orange
                    elif event.key == pygame.K_o and self.type != RTKType.ORANGE:
                        self.type = RTKType.ORANGE
                        self.get_logger().info("Ready to record ORANGE cones")
                    # Only change to big_orange if not already big_orange
                    elif event.key == pygame.K_s and self.type != RTKType.BIG_ORANGE:
                        self.type = RTKType.BIG_ORANGE
                        self.get_logger().info("Ready to record BIG_ORANGE cones")
                    elif event.key == pygame.K_c and self.type != RTKType.CAR_START:
                        self.type = RTKType.CAR_START
                        self.get_logger().info("Ready to record CAR_START")
                    elif event.key == pygame.K_d:
                        with open(self.map_path, "r+") as f:
                            lines = f.readlines()
                            assert len(lines) > 0, "File is empty?! Where's the header??"
                            if len(lines) == 1:
                                self.get_logger().info("No measurements to delete!")
                                return
                            else:
                                self.get_logger().info("Removing last cone measurement!")
                                f.seek(0)
                                f.write("".join(lines[:-1]))
                                f.truncate()
                                if lines[-1].split(",")[0] == "car_start":
                                    self.car_start_count -= 1
                        self.update()

            # If the space bar is released, then stop logging messages
            elif event.type == pygame.KEYUP and event.key == pygame.K_SPACE:
                if self.type is not None and self.log:
                    self.get_logger().info("Stop acummulation of messages")
                    self.log = False

        # Update pygame display
        pygame.display.update()

    def fix_callback(self, msg):
        # Check msg status
        if msg.status.status == NavSatStatus.STATUS_GBAS_FIX:
            self.get_logger().info("RTK fix achieved!", once=True, skip_first=True)

        # If self.log is true, capture message and store it
        if self.log:
            # Add message to buffer
            self.fix_buff = np.vstack((self.fix_buff, [msg.latitude, msg.longitude]))
            cov = np.array([
                [msg.position_covariance[4], msg.position_covariance[3]],
                [msg.position_covariance[1], msg.position_covariance[0]]])
            self.cov_buff = np.vstack((self.cov_buff, [cov]))
            self.get_logger().info(f"Accumulated {self.fix_buff.shape[0]} messages")
        # If self.log is false but we have stuff in the buffer, save it then clear
        elif not self.log and self.fix_buff.shape[0] > 0:
            avg_fix = np.mean(self.fix_buff, axis=0)
            avg_cov = np.mean(self.cov_buff, axis=0)
            self.get_logger().debug(f"Avg cone position: {avg_fix}")
            self.get_logger().debug(f"Avg cone std: {avg_cov}")
            self.fix_buff = np.array([]).reshape(0, 2)
            self.cov_buff = np.array([]).reshape(0, 2, 2)

            # Check if var is too big
            if avg_cov[0, 0] > self.thresh or avg_cov[1, 1] > self.thresh:
                self.get_logger().warn(
                    f"Accuracy is above threshold: {avg_cov[0,0]}, {avg_cov[1,1]}")

            # Append to csv if not in update mode
            if not self.update_mode:
                fix = f"{self.type},{avg_fix[0]},{avg_fix[1]},"
                fix += f"{avg_cov[0,0]},{avg_cov[1,1]},{avg_cov[0,1]}\n"
                with open(self.map_path, "a") as f:
                    f.write(fix)
                self.get_logger().info(f"Saved {self.type} cone at {avg_fix}")
                if self.type == RTKType.CAR_START:
                    self.car_start_count += 1
                self.update()
            else:
                df = pd.read_csv(self.map_path)
                df_np = df[['latitude', 'longitude']].to_numpy()

                if df_np.shape[0] == 0:
                    self.get_logger().warn("No cones to update!")
                    return

                # Calculate closest point
                idxs = np.argsort(np.sum((df_np - avg_fix)**2, axis=1))
                for idx in idxs:
                    if df.loc[idx].type == str(self.type):
                        prev = df.loc[idx][['latitude', 'longitude']].to_numpy()
                        df.loc[idx] = {
                            'type': str(self.type),
                            'latitude': avg_fix[0],
                            'longitude': avg_fix[1],
                            'latitude_var': avg_cov[0, 0],
                            'longitude_var': avg_cov[1, 1],
                            'covariance': avg_cov[0, 1]}
                        break

                # Over-write with new cone position
                df.to_csv(self.map_path, index=False)
                self.get_logger().info(f"Updated {self.type} cone at {prev} to {avg_fix}")
                self.update()

    def update(self):
        # Update matplotlib figure screen
        fig, ax = self.viz.run(True)
        ax.set_xticklabels([])
        ax.set_yticklabels([])
        ax.set_ylabel("")
        ax.set_xlabel("")
        # fig.tight_layout()
        fig.canvas.draw()
        # Use the fig as a pygame.Surface
        self.screen.blit(fig, (0, 0))
        plt.close(fig)


def main(args=None):
    rclpy.init(args=args)

    rtk_mapper = RTKMapper()
    try:
        rclpy.spin(rtk_mapper)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
