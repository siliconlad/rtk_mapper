from typing import List

import numpy as np
import pandas as pd
from rtk_mapper.map import RTKMap, Marker, MarkerType


class InvalidCSVFormat(RuntimeError):
    """Raised if CSV file has the wrong structure."""


class CSVFormat:
    """Base class for reading a CSV file."""

    @staticmethod
    def load_csv(path: str) -> RTKMap:
        """
        Load map from CSV file.

        :param path: path to csv file
        :returns: RTKMap
        """
        raise NotImplementedError

    @staticmethod
    def save_csv(rtk_map: RTKMap, path: str) -> None:
        """
        Save map as a CSV file.

        :param rtk_map: RTK map
        :param path: path to save csv file
        """
        raise NotImplementedError

    @staticmethod
    def get_marker_type(m_type: str) -> MarkerType:
        """
        Returns a specific MarkerType based on string input.

        :param m_type: marker type name as a string
        :returns: MarkerType of the appropriate type
        :raises ValueError: raised if type name is not one of
                            [blue, yellow, orange, big_orange, car_start].
        """
        if m_type == "blue":
            return MarkerType.BLUE
        if m_type == "yellow":
            return MarkerType.YELLOW
        if m_type == "big_orange":
            return MarkerType.BIG_ORANGE
        if m_type == "orange":
            return MarkerType.ORANGE
        if m_type == "car_start":
            return MarkerType.CAR_START

        raise ValueError(f"Invalid marker string: {m_type}")

    @staticmethod
    def get_marker_str(m_type: MarkerType) -> str:
        """
        Returns a specific string based on MarkerType input.

        :param m_type: MarkerType
        :returns: string for the appropriate type
        :raises ValueError: raised if type name is not a valid MarkerType
        """
        if m_type == MarkerType.BLUE:
            return "blue"
        if m_type == MarkerType.YELLOW:
            return "yellow"
        if m_type == MarkerType.BIG_ORANGE:
            return "big_orange"
        if m_type == MarkerType.ORANGE:
            return "orange"
        if m_type == MarkerType.CAR_START:
            return "car_start"

        raise ValueError(f"Invalid marker type: {m_type}")

    def __str__(self):
        raise NotImplementedError


class EUFSFormat(CSVFormat):
    """
    Conversion tool between CSV in EUFS Format (eufs_sim) and RTKMap object.

    EUFS Format:
        tag: type of marker [blue, yellow, big_orange, orange, car_start]
        x: x position
        y: y position
        direction: angle from the x-axis
        x_variance: variance in x
        y_variance: variance in y
        xy_covariance: covariance in xy
    """
    fmt: List[str] = ['tag', 'x', 'y', 'direction', 'x_variance', 'y_variance', 'xy_covariance']

    @staticmethod
    def load_csv(path: str) -> RTKMap:
        """
        Load map from CSV file in EUFS format.

        :param path: path to csv file
        :returns: RTKMap
        """

        # Read csv
        df: pd.DataFrame = pd.read_csv(path)

        # Check csv headers
        if list(df.columns) != EUFSFormat.fmt:
            raise InvalidCSVFormat("CSV does not have the correct headers!")

        rtk_map: RTKMap = RTKMap()
        for _, row in df.iterrows():
            # Ignore direction because it makes no difference for the map
            pos: np.ndarray = np.array([row['x'], row['y']])
            cov: np.ndarray = np.array(
                [row['x_variance'], row['xy_covariance'], row['xy_covariance'], row['y_variance']])
            m_type: MarkerType = EUFSFormat.get_marker_type(row['tag'])
            rtk_map.add_marker(Marker(pos, cov, m_type))
        return rtk_map

    @staticmethod
    def save_csv(rtk_map: RTKMap, path: str) -> None:
        """
        Save map as a CSV file.

        :param rtk_map: RTK map
        :param path: path to save csv file
        """
        with open(path, "w") as file:
            file.write(",".join(EUFSFormat.fmt) + "\n")  # Write csv header
            for marker in rtk_map.markers:
                line: List[str] = [
                    EUFSFormat.get_marker_str(marker.type),
                    str(marker.pos[0]),
                    str(marker.pos[1]),
                    str(marker.cov[0]),
                    str(marker.cov[3]),
                    str(marker.cov[1])
                ]
                file.write(",".join(line) + "\n")

    def __str__(self):
        return "eufs"


class RTKFormat(CSVFormat):
    """
    Conversion tool between CSV in rtk_mapper format and RTKMap object.

    rtk_mapper Format:
        type: type of marker [blue, yellow, big_orange, orange, car_start]
        latitude: GPS latitude
        longitude: GPS longitude
        latitude_var: latitude variance
        longitude_var: longitude variance
        covariance: latitude-longitude covariance
    """
    fmt: List[str] = ['type', 'latitude', 'longitude', 'latitude_var', 'longitude_var',
                      'covariance']

    @staticmethod
    def load_csv(path: str) -> RTKMap:
        """
        Load map from CSV file in rtk_mapper format.

        :param path: path to csv file
        :returns: RTKMap
        """

        # Read csv
        df: pd.DataFrame = pd.read_csv(path)

        # Check csv headers
        if list(df.columns) != RTKFormat.fmt:
            raise InvalidCSVFormat("CSV does not have the correct headers!")

        rtk_map: RTKMap = RTKMap()
        for _, row in df.iterrows():
            pos: np.ndarray = np.array([row['longitude'], row['latitude']])
            cov: np.ndarray = np.array(
                [row['longitude_var'], row['covariance'], row['covariance'], row['latitude_var']])
            m_type: MarkerType = RTKFormat.get_marker_type(row['type'])
            rtk_map.add_marker(Marker(pos, cov, m_type))
        return rtk_map

    @staticmethod
    def save_csv(rtk_map: RTKMap, path: str) -> None:
        """
        Save map as a CSV file.

        :param rtk_map: RTK map
        :param path: path to save csv file
        """
        with open(path, "w") as file:
            file.write(",".join(RTKFormat.fmt) + "\n")  # Write csv header
            for marker in rtk_map.markers:
                line: List[str] = [
                    RTKFormat.get_marker_str(marker.type),
                    str(marker.pos[1]),
                    str(marker.pos[0]),
                    str(marker.cov[0]),
                    str(marker.cov[3]),
                    str(marker.cov[1])
                ]
                file.write(",".join(line) + "\n")

    def __str__(self):
        return "rtk"
