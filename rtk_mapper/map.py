from __future__ import annotations

import warnings

import matplotlib.axes
import numpy as np
from enum import Enum

from typing import List


class NotEnoughOrangeCones(RuntimeError):
    """Raised when map has not enough orange cones."""
    pass


class NoCarStartMarker(RuntimeError):
    """Raised when map has no car start marker."""
    pass


class InUpdateMode(RuntimeError):
    """Raised when RTKMap is in update mode"""
    pass


class NotInUpdateMode(RuntimeError):
    """Raised when RTKMap is not in update mode"""
    pass


class NoMarkers(IndexError):
    """Raised when no markers are left and deletion is attempted."""
    pass


class MarkerType(str, Enum):
    """Marker types with associated hex color."""
    hex_code: str

    # https://rednafi.github.io/reflections/add-additional-attributes-to-enum-members-in-python.html
    def __new__(cls, title: str, hex_code: str = "") -> MarkerType:
        obj = str.__new__(cls, title)
        obj._value_ = title

        obj.hex_code = hex_code
        return obj

    BLUE = ("blue", "#0000ff")
    YELLOW = ("yellow", "#ecef1f")
    ORANGE = ("orange", "#ff8c00")
    BIG_ORANGE = ("big_orange", "#ff8c00")
    UNKNOWN = ("unknown", "#636363")
    CAR_START = ("car_start", "#36a538")


class Marker:
    def __init__(self, pos: np.ndarray, cov: np.ndarray, m_type: MarkerType):
        self.pos: np.ndarray = np.array(pos).reshape(-1,)
        self.cov: np.ndarray = np.array(cov).reshape(-1,)
        self.type: MarkerType = m_type


class RTKMap:
    def __init__(self):
        self.update_mode: bool = False  # Determines if Map is in update mode
        self.markers: List[Marker] = []  # Stores the list of markers representing the map
        self.selected: int = 0  # Index of selected marker

    def add_marker(self, m: Marker) -> None:
        """
        Adds marker to map.

        :param m: marker to add
        :raises InUpdateMode: raised when in update mode
        """
        if self.update_mode:
            raise InUpdateMode("Cannot add marker while in update mode.")

        self.markers.append(m)
        self.selected = len(self.markers) - 1

    def remove_marker(self) -> None:
        """
        Removes marker. Which one depends on the mode. If in update mode, removes the selected
        marker. Otherwise, removes the last added marker. In both cases, the new selected
        marker is the previous marker.

        :raises NoMarkers: raised when no markers are in the map
        """
        if not self.markers:
            raise NoMarkers("No markers to remove!")

        idx: int = self.selected if self.update_mode else -1
        self.markers.pop(idx)
        self.select_prev_marker()  # Select previous marker to ensure index is always valid

    def enter_update_mode(self) -> None:
        """Enters update mode. Does nothing if already in update mode."""
        self.update_mode = True

    def exit_update_mode(self) -> None:
        """Exits update mode. Does nothing if not in update mode."""
        self.update_mode = False

    def select_next_marker(self) -> None:
        """
        Selects next marker in the sequence in which the markers were added to the map.
        Wraps around to the beginning if the currently selected marker is at the end.

        :raises NotInUpdateMode: raised if not in update mode.
        """
        if not self.update_mode:
            raise NotInUpdateMode("Must be in update mode to select next marker.")
        self.selected = (self.selected + 1) % len(self.markers)

    def select_prev_marker(self) -> None:
        """
        Selects previous marker in the sequence in which the markers were added to the map.
        Wraps around to the end if currently selected marker is at index 0.

        :raises NotInUpdateMode: raised if not in update mode.
        """
        if not self.update_mode:
            raise NotInUpdateMode("Must be in update mode to select prev marker.")
        self.selected = (self.selected + len(self.markers) - 1) % len(self.markers)

    def update_marker(self, m: Marker) -> None:
        """
        Updates the currently selected marker with the new marker.

        :param m: new marker to replace selected marker.
        :raises NotInUpdateMode: raised if not in update mode.
        """
        if not self.update_mode:
            raise NotInUpdateMode("Must be in update mode to select prev marker.")
        self.markers[self.selected] = m

    def normalize(self) -> None:
        """
        Calculates and performs a linear transformation of all markers so that car_start
        is at the origin and the finish line marked by the orange cones are in x-direction.

        :raises NoCarStartMarker: Normalization cannot occur without a CAR_START marker.
        :raises NotEnoughOrangeCones: Normalization cannot occur without at least 1 orange cone.
        """
        car_start: Marker | None = None
        big_orange: np.ndarray = np.array([]).reshape((0, 2))
        for marker in self.markers:
            if marker.type == MarkerType.CAR_START:
                car_start = marker
            if marker.type == MarkerType.BIG_ORANGE:
                big_orange = np.vstack((big_orange, marker.pos))

        # Need car start to determine origin
        if car_start is None:
            raise NoCarStartMarker("Map does not contain CAR_START marker. Cannot normalize map.")

        # Need at least one orange cone to do normalization
        if big_orange.shape[0] < 1:
            raise NotEnoughOrangeCones("Not have enough BIG_ORANGE cones. Cannot normalize map.")

        # Some tracks may only have two cones (e.g Skidpad).
        if big_orange.shape[0] < 2:
            warnings.warn("Only 1 BIG_ORANGE cone.", RuntimeWarning)

        # Find the 4 closest big_orange cones (i.e finish line)
        sorted_oranges: np.ndarray = np.argsort(np.sum((big_orange - car_start)**2, axis=1))
        if len(sorted_oranges) > 4:
            sorted_oranges = sorted_oranges[:4]
        # Calculate the average of the 4 big_orange cones
        orange_center: np.ndarray = np.mean(big_orange[sorted_oranges], axis=0)

        # Calculate transformation of cones so that car_start is the origin and the
        # center of the finish line is in the x-direction
        orange_center_p: np.ndarray = orange_center - car_start
        theta: float = np.arctan2(orange_center_p[1], orange_center_p[0])
        R: np.ndarray = np.array([[np.cos(theta), np.sin(theta)], [-np.sin(theta), np.cos(theta)]])

        # Transform every marker
        for marker in self.markers:
            marker.pos = (marker.pos - car_start) @ R.T


def main():
    """
    Takes a path to CSV and plots it using RTKMap.
    """
    pass


if __name__ == '__main__':
    main()
