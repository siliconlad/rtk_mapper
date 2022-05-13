from copy import deepcopy

import numpy as np
import utm

from rtk_mapper.map import RTKMap, Marker


def utm_transform(map: RTKMap, copy: bool = False) -> RTKMap:
    """Convert markers in map to UTM coordinates."""
    m: RTKMap = deepcopy(map) if copy else map
    for i, marker in enumerate(m.markers):
        m.markers[i] = utm_transform_marker(marker)


def utm_transform_marker(marker: Marker, copy: bool = False) -> Marker:
    """Convert marker to UTM coordinates."""
    # Marker stores pos as (longitude, latitude)
    m: Marker = deepcopy(marker) if copy else marker
    utm_coord = utm.from_latlon(m.pos[1], m.pos[0])
    # utm_coord is (easting, northing,...)
    m.pos = np.array(utm_coord[0], utm_coord[1])
    return m
