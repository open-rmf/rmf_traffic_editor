from pyproj import Transformer
from pyproj import CRS
import math


class WGS84Transform:
    """Transforms between WGS84 points and transverse mercator planes"""

    def __init__(self, crs_name, offset):
        print(f'WGS84Transform({crs_name}, {offset})')
        self.crs_name = crs_name
        self.x = offset[0]
        self.y = offset[1]
        self.rotation = 0
        self.wgs84_to_tm = \
            Transformer.from_crs("EPSG:4326", self.crs_name)

    def transform_point(self, p):
        lon = p[0]
        lat = p[1]

        (tm_northing, tm_easting) = \
            self.wgs84_to_tm.transform(lat, lon)

        # print(f'lat={lat} lon={lon} => ({tm_easting}, {tm_northing})')

        return (tm_easting, tm_northing)
