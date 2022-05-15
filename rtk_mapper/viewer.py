from __future__ import annotations

import os
import signal
import sys

import matplotlib
import matplotlib.backends.backend_agg as agg
import matplotlib.figure
import matplotlib.pyplot as plt
import pygame
import pygame.event
import pygame.locals
import pylab
import rclpy
from rclpy.node import Node

from rtk_mapper.csv import RTKFormat, EUFSFormat, InvalidCSVFormat
from rtk_mapper.map import RTKMap
from rtk_mapper.plot import RTKPlotter
from rtk_mapper.utm import utm_transform

# Matplotlib magic to get it to work with pygame
# See https://www.pygame.org/wiki/MatplotlibPygame
matplotlib.use("Agg")


class RTKViewer(Node):
    def __init__(self, name="rtk_mapper"):
        super().__init__(name)
        self.EUFS_MASTER = os.environ["EUFS_MASTER"]

        # Initialize pygame
        pygame.init()
        self.window = pygame.display.set_mode((700, 700), pygame.locals.DOUBLEBUF)
        self.screen: pygame.Surface = pygame.display.get_surface()

        # Load CSV
        if (map_path := self.declare_parameter("map", "").value) == "":
            raise ValueError("The map parameter must be set.")
        try:
            self.map: RTKMap = RTKFormat.load_csv(map_path)
            self.raw_gps: bool = True
        except InvalidCSVFormat:
            try:
                self.map: RTKMap = EUFSFormat.load_csv(map_path)
                self.raw_gps: bool = False
            except InvalidCSVFormat:
                self.get_logger().error("Invalid CSV format.")
        self.update()

        # ROS Timer (keyboard loop @ 100Hz)
        self.timer_ = self.create_timer(0.01, self.loop)

        # Finish by printing instructions on how to use the program
        self.get_logger().info("rtk_viewer is ready!")

    def shutdown(self) -> None:
        """Shuts down the program."""
        self.get_logger().info("Shutting down...")
        pygame.quit()
        signal.raise_signal(signal.SIGINT)
        sys.exit()

    def loop(self) -> None:
        """Main pygame loop which handles keyboard input."""
        event: pygame.event.EventType
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q:
                    self.shutdown()
        self.update()

    def update(self) -> None:
        """
        Update pygame display with new plot.
        """
        # Create figure
        fig: matplotlib.figure.Figure = pylab.figure(figsize=[7, 7], dpi=100)
        ax = fig.gca()

        # Plot map in UTM because it looks better
        if self.raw_gps:
            RTKPlotter.plot(utm_transform(self.map, copy=True), ax)
        else:
            RTKPlotter.plot(self.map, ax)
        fig.tight_layout()

        # Matplotlib magic to get it to work with pygame
        # See https://www.pygame.org/wiki/MatplotlibPygame
        canvas = agg.FigureCanvasAgg(fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        raw_data = renderer.tostring_rgb()
        size = canvas.get_width_height()
        surf = pygame.image.fromstring(raw_data, size, "RGB")
        self.screen.blit(surf, (0, 0))
        pygame.display.flip()

        # Destroy figure
        plt.close(fig)


def main(args=None):
    rclpy.init(args=args)

    rtk_mapper = RTKViewer()
    try:
        rclpy.spin(rtk_mapper)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
