from typing import List

import numpy as np
import pandas as pd

from rtk_mapper.map import RTKMap, Marker, MarkerType


class InvalidCSVFormat(RuntimeError):
    """Raised if CSV file has the wrong structure."""
    pass


class CSVFormat:
    """Base class for reading a CSV file."""

    @staticmethod
    def load_csv(path: str) -> RTKMap:
        raise NotImplementedError

    @staticmethod
    def save_csv(m: RTKMap, path: str) -> None:
        raise NotImplementedError

    @staticmethod
    def get_marker_type(t: str) -> MarkerType:
        """
        Returns a specific MarkerType based on string input.

        :param t: marker type name as a string
        :returns: MarkerType of the appropriate type
        :raises ValueError: raised if type name is not one of
                            [blue, yellow, orange, big_orange, car_start].
        """
        if t == "blue":
            return MarkerType.BLUE
        elif t == "yellow":
            return MarkerType.YELLOW
        elif t == "big_orange":
            return MarkerType.BIG_ORANGE
        elif t == "orange":
            return MarkerType.ORANGE
        elif t == "car_start":
            return MarkerType.CAR_START

        raise ValueError(f"Invalid marker string: {t}")

    @staticmethod
    def get_marker_str(t: MarkerType) -> str:
        """
        Returns a specific string based on MarkerType input.

        :param t: MarkerType
        :returns: string for the appropriate type
        :raises ValueError: raised if type name is not a valid MarkerType
        """
        if t == MarkerType.BLUE:
            return "blue"
        elif t == MarkerType.YELLOW:
            return "yellow"
        elif t == MarkerType.BIG_ORANGE:
            return "big_orange"
        elif t == MarkerType.ORANGE:
            return "orange"
        elif t == MarkerType.CAR_START:
            return "car_start"

        raise ValueError(f"Invalid marker type: {t}")

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

        m: RTKMap = RTKMap()
        for i, r in df.iterrows():
            # Ignore direction because it makes no difference for the map
            pos: np.ndarray = np.array([r['x'], r['y']])
            cov: np.ndarray = np.array([r['x_variance'], r['xy_covariance'], r['xy_covariance'], r['y_variance']])
            m_type: MarkerType = EUFSFormat.get_marker_type(r['tag'])
            m.add_marker(Marker(pos, cov, m_type))
        return m

    @staticmethod
    def save_csv(m: RTKMap, path: str) -> None:
        """
        Save map as a CSV file.

        :param m: RTK map
        :param path: path to save csv file
        """
        with open(path, "w") as f:
            f.write(",".join(EUFSFormat.fmt) + "\n")  # Write csv header
            for marker in m.markers:
                line: List[str] = [
                    EUFSFormat.get_marker_str(marker.type),
                    str(marker.pos[0]),
                    str(marker.pos[1]),
                    str(marker.cov[0]),
                    str(marker.cov[3]),
                    str(marker.cov[1])
                ]
                f.write(",".join(line) + "\n")

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
    fmt: List[str] = ['type', 'latitude', 'longitude', 'latitude_var', 'longitude_var', 'covariance']

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

        m: RTKMap = RTKMap()
        for i, r in df.iterrows():
            pos: np.ndarray = np.array([r['longitude'], r['latitude']])
            cov: np.ndarray = np.array([r['longitude_var'], r['covariance'], r['covariance'], r['latitude_var']])
            m_type: MarkerType = RTKFormat.get_marker_type(r['type'])
            m.add_marker(Marker(pos, cov, m_type))
        return m

    @staticmethod
    def save_csv(m: RTKMap, path: str) -> None:
        """
        Save map as a CSV file.

        :param m: RTK map
        :param path: path to save csv file
        """
        with open(path, "w") as f:
            f.write(",".join(RTKFormat.fmt) + "\n")  # Write csv header
            for marker in m.markers:
                line: List[str] = [
                    RTKFormat.get_marker_str(marker.type),
                    str(marker.pos[0]),
                    str(marker.pos[1]),
                    str(marker.cov[0]),
                    str(marker.cov[3]),
                    str(marker.cov[1])
                ]
                f.write(",".join(line) + "\n")

    def __str__(self):
        return "rtk"
