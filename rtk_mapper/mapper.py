from __future__ import annotations

import os
import signal
import sys
from datetime import datetime
from os import path

import matplotlib
import matplotlib.backends.backend_agg as agg
import matplotlib.figure
import matplotlib.pyplot as plt
import numpy as np
import pygame
import pygame.event
import pygame.locals
import pylab
import rclpy
from rclpy.logging import LoggingSeverity
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from rtk_mapper.csv import RTKFormat, EUFSFormat
from rtk_mapper.map import RTKMap, Marker, MarkerType, NoMarkers, NotInUpdateMode, NoCarStartMarker
from rtk_mapper.plot import RTKPlotter
from rtk_mapper.utm import utm_transform

# Matplotlib magic to get it to work with pygame
# See https://www.pygame.org/wiki/MatplotlibPygame
matplotlib.use("Agg")


class RTKMapper(Node):
    def __init__(self, name="rtk_mapper"):
        super().__init__(name)
        self.EUFS_MASTER = os.environ["EUFS_MASTER"]

        # Initialize pygame
        pygame.init()
        self.window = pygame.display.set_mode((700, 700), pygame.locals.DOUBLEBUF)
        self.screen: pygame.Surface = pygame.display.get_surface()

        # State
        self.map: RTKMap = RTKMap()
        self.car_start_count: int = 0
        self.accumulate: bool = False
        self.m_type: MarkerType | None = None
        self.m_type_str: str = ""
        self.fix_buff: np.ndarray = np.array([]).reshape((0, 2))
        self.cov_buff: np.ndarray = np.array([]).reshape((0, 2, 2))
        # Save path to where the map will be stored
        date = datetime.now().strftime('%Y_%m_%d-%H_%M_%S')
        self.map_path: str = path.join(self.EUFS_MASTER, "map", f"rtk_map_{date}.csv")
        self.utm_map_path: str = path.join(self.EUFS_MASTER, "map", f"rtk_map_{date}_utm.csv")

        # ROS Parameters
        self.thresh: float = self.declare_parameter("threshold", 0.0002).value
        self.get_logger().info(f"Threshold is {self.thresh}")
        if self.declare_parameter("debug", False).value:
            self.get_logger().set_level(LoggingSeverity.DEBUG)
            self.get_logger().debug("Debug logging enabled")

        # ROS Subscriber
        self.sub_ = self.create_subscription(NavSatFix, "/fix", self.fix_cb, 1)

        # ROS Timer (keyboard loop @ 100Hz)
        self.timer_ = self.create_timer(0.01, self.loop)

        # Print empty figure
        self.update()

        # Finish by printing instructions on how to use the program
        self.get_logger().info("rtk_mapper is ready!")
        self.print_help()

    def print_help(self) -> None:
        """Print help"""
        self.get_logger().info("--------")
        self.get_logger().info("Press b for BLUE, y for YELLOW, o for ORANGE, s for BIG_ORANGE")
        self.get_logger().info("Press c to enter CAR START position")
        self.get_logger().info("Press u to enter UPDATE MODE")
        self.get_logger().info("Press d to delete last measurement, or selected cone in UPDATE MODE")
        self.get_logger().info("Press TAB in UPDATE MODE to select cone")
        self.get_logger().info("Press press space to record messages")
        self.get_logger().info("Press press q to quit")
        self.get_logger().info("--------")

    def shutdown(self) -> None:
        """Shuts down the program."""
        self.get_logger().info("Shutting down...")

        # Save rtk map
        RTKFormat.save_csv(self.map, self.map_path)
        self.get_logger().info(f"Saved RTK map to {self.map_path}")

        # Save utm map
        utm_map: RTKMap = utm_transform(self.map, copy=False)
        try:
            utm_map.normalize()
            EUFSFormat.save_csv(utm_map, self.utm_map_path)
            self.get_logger().info(f"Saved UTM map to {self.utm_map_path}")
        except NoMarkers:
            self.get_logger().error("No markers in map. Cannot normalize.")
        except NoCarStartMarker:
            self.get_logger().error("No CAR_START marker. Cannot normalize map.")

        # Quit cleanly
        pygame.quit()
        signal.raise_signal(signal.SIGINT)
        sys.exit()

    def start_accumulation(self) -> None:
        """
        Enable the accumulation of NavSatFix messages as long as marker type is set.
        Prevent the recording of multiple car_start markers.
        """
        have_car_start: bool = self.m_type == MarkerType.CAR_START and self.car_start_count > 0
        if have_car_start and not self.map.update_mode:
            self.get_logger().warn("Already recorded car_start!")
        elif self.m_type is None:
            self.get_logger().warn("Please select a cone color before accumulating!")
        else:
            self.get_logger().info("Start accumulation of messages!")
            self.accumulate = True

    def stop_accumulation(self) -> None:
        """Stop accumulation of messages."""
        if self.accumulate:
            self.get_logger().info("Stop accumulation of messages")
            self.accumulate = False

    def toggle_update_mode(self) -> None:
        """Toggle update mode."""
        if self.map.update_mode:
            self.map.exit_update_mode()
            self.get_logger().info("Exited UPDATE mode.")
        else:
            self.map.enter_update_mode()
            self.get_logger().info("Entered UPDATE mode.")

    def set_marker(self, m_type: MarkerType) -> None:
        """Set the type of marker to record."""
        if self.m_type != m_type:
            self.m_type = m_type
            self.m_type_str = RTKFormat.get_marker_str(self.m_type)
            self.get_logger().info(f"Ready to record {self.m_type_str.upper()} cones")

    def delete(self) -> None:
        """Delete marker"""
        try:
            self.map.remove_marker()
        except NoMarkers:
            self.get_logger().warn("No markers to delete!")

    def select_next_marker(self) -> None:
        try:
            self.map.select_next_marker()
        except NotInUpdateMode:
            self.get_logger().warn("Must be in update mode to select next marker.")
        except NoMarkers:
            self.get_logger().warn("Map is empty; next marker cannot be selected.")

    def select_prev_marker(self) -> None:
        try:
            self.map.select_prev_marker()
        except NotInUpdateMode:
            self.get_logger().warn("Must be in update mode to select prev marker.")
        except NoMarkers:
            self.get_logger().warn("Map is empty; prev marker cannot be selected.")

    def loop(self) -> None:
        """Main pygame loop which handles keyboard input."""
        event: pygame.event.EventType
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.shutdown()
                elif event.unicode == ' ':  # space key
                    self.start_accumulation()
                elif event.unicode == 'u':
                    self.toggle_update_mode()
                elif event.unicode == 'b':
                    self.set_marker(MarkerType.BLUE)
                elif event.unicode == 'y':
                    self.set_marker(MarkerType.YELLOW)
                elif event.unicode == 'o':
                    self.set_marker(MarkerType.ORANGE)
                elif event.unicode == 'O':
                    self.set_marker(MarkerType.BIG_ORANGE)
                elif event.unicode == 'c':
                    self.set_marker(MarkerType.CAR_START)
                elif event.unicode == 'd':
                    self.delete()
                elif event.key == pygame.K_TAB and event.mod == 0:
                    self.select_next_marker()
                elif event.key == pygame.K_TAB and event.mod == 1:
                    self.select_prev_marker()
            elif event.type == pygame.KEYUP and event.key == pygame.K_SPACE:
                self.stop_accumulation()
        self.update()

    def fix_cb(self, msg: NavSatFix) -> None:
        """
        Callback for NavSatFix subscriber.

        :param msg: sensor_msgs/NavSatFix message
        """
        if self.accumulate:
            # Add message to buffer
            self.fix_buff = np.vstack((self.fix_buff, [msg.longitude, msg.latitude]))
            cov: list = [
                msg.position_covariance[0], msg.position_covariance[3],
                msg.position_covariance[1], msg.position_covariance[4]]
            self.cov_buff = np.vstack((self.cov_buff, [cov]))
            self.get_logger().info(f"Accumulated {self.fix_buff.shape[0]} messages")
        # If self.log is false, but we have stuff in the buffer, create new marker
        elif not self.accumulate and self.fix_buff.shape[0] > 0:
            avg_fix = np.mean(self.fix_buff, axis=0)
            avg_cov = np.mean(self.cov_buff, axis=0)
            self.get_logger().debug(f"Avg cone position: {avg_fix}")
            self.get_logger().debug(f"Avg cone cov: {avg_cov}")
            # Clear buffers
            self.fix_buff = np.array([]).reshape((0, 2))
            self.cov_buff = np.array([]).reshape((0, 2, 2))

            # Check if covariance is above threshold
            if avg_cov[0, 0] > self.thresh or avg_cov[1, 1] > self.thresh:
                self.get_logger().warn(f"Accuracy is above thresh: {avg_cov[0,0]}, {avg_cov[1,1]}")

            # Create new marker if not in update mode
            m = Marker(avg_fix, avg_cov, self.m_type)
            if not self.map.update_mode:
                self.map.add_marker(m)
                if self.m_type == MarkerType.CAR_START:
                    self.car_start_count += 1
                self.get_logger().info(f"Saved {self.m_type_str} cone at {avg_fix}")
            else:
                self.map.update_marker(m)

    def update(self) -> None:
        """
        Update pygame display with new plot.
        """
        # Create figure
        fig: matplotlib.figure.Figure = pylab.figure(figsize=[7, 7], dpi=100)
        ax = fig.gca()

        # Plot map in UTM because it looks better
        RTKPlotter.plot(utm_transform(self.map, copy=True), ax)
        fig.tight_layout()

        # Matplotlib magic to get it to work with pygame
        # See https://www.pygame.org/wiki/MatplotlibPygame
        canvas = agg.FigureCanvasAgg(fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.tostring_rgb()
        size = canvas.get_width_height()
        surf = pygame.image.fromstring(raw_data, size, "RGB")
        self.screen.blit(surf, (0,0))
        pygame.display.flip()

        # Destroy figure
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
