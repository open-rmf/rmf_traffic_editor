from pyproj import Transformer
from pyproj import CRS
import math


class WebMercatorTransform:
    """Transforms between Web Mercator points and transverse mercator planes"""

    def __init__(self, crs_name):
        print(f'WebMercatorTransform({crs_name})')
        self.crs_name = crs_name
        self.offset = (0, 0)
        self.web_mercator_to_wgs84 = \
            Transformer.from_crs("EPSG:3857", "EPSG:4326")
        self.web_mercator_to_tm = \
            Transformer.from_crs("EPSG:3857", self.crs_name)

    def set_offset(self, new_offset):
        self.offset = new_offset

    def transform_point(self, p):
        # first convert from "Web Mercator" (256, 256) scale to meters

        R = 6378137  # wgs84 equatorial radius
        meters_east = R * math.pi * (p[0] - 128) / 128
        # because our legacy code is all in image coordinates, the Y coord
        # is negated... this step will flip it back
        # TODO: revisit this now that loading is not always flipped...
        meters_north = R * math.pi * (128 + p[1]) / 128

        (lat, lon) = \
            self.web_mercator_to_wgs84.transform(meters_east, meters_north)

        (tm_northing, tm_easting) = \
            self.web_mercator_to_tm.transform(meters_east, meters_north)

        tm_easting -= self.offset[0]
        tm_northing -= self.offset[1]

        return (tm_easting, tm_northing)
