from pyproj import Transformer
from pyproj import CRS
import math


class WebMercatorTransform:
    """Transforms between Web Mercator points and transverse mercator planes"""

    def __init__(self):
        # crs_4326 = CRS.from_epsg(4326)  # also known as WGS84...
        self.web_mercator_to_wgs84 = \
            Transformer.from_crs("EPSG:3857", "EPSG:4326")
        self.web_mercator_to_svy21 = \
            Transformer.from_crs("EPSG:3857", "EPSG:3414")

    def transform_point(self, p):
        # first convert from "Web Mercator" (256, 256) scale to meters

        R = 6378137  # wgs84 equatorial radius
        meters_east = R * math.pi * (p[0] - 128) / 128
        # because our legacy code is all in image coordinates, the Y coord
        # is negated... this step will flip it back
        meters_north = R * math.pi * (128 + p[1]) / 128

        (lat, lon) = \
            self.web_mercator_to_wgs84.transform(meters_east, meters_north)

        # now we need to choose a TM plane. In the future we should be more
        # generic, but for now let's start by using SVY21
        (svy21_northing, svy21_easting) = \
            self.web_mercator_to_svy21.transform(meters_east, meters_north)

        # crs_svy21 = CRS.from_epsg(3414)
        # svy21_false_easting = crs_svy21.to_dict()['x_0']
        # svy21_false_northing = crs_svy21.to_dict()['y_0']
        # print(f'offset: {svy21_false_easting}, {svy21_false_northing}')

        return (svy21_easting, svy21_northing)
