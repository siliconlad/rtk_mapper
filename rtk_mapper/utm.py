from copy import deepcopy

import numpy as np
import utm

from rtk_mapper.map import RTKMap, Marker


def utm_transform(rtk_map: RTKMap, copy: bool = False) -> RTKMap:
    """Convert markers in map to UTM coordinates."""
    utm_map: RTKMap = deepcopy(rtk_map) if copy else rtk_map
    for i, marker in enumerate(utm_map.markers):
        utm_map.markers[i] = utm_transform_marker(marker)
    return utm_map


def utm_transform_marker(marker: Marker, copy: bool = False) -> Marker:
    """Convert marker to UTM coordinates."""
    # Marker stores pos as (longitude, latitude)
    utm_marker: Marker = deepcopy(marker) if copy else marker
    utm_coord = utm.from_latlon(utm_marker.pos[1], utm_marker.pos[0])
    # utm_coord is (easting, northing,...)
    utm_marker.pos = np.array([utm_coord[0], utm_coord[1]])
    return utm_marker
