from enum import Enum


class CoordinateSystem(Enum):
    reference_image = 1
    web_mercator = 2
    cartesian_meters = 3
    wgs84 = 4

    def y_flip_scalar(self):
        if self == CoordinateSystem.reference_image:
            return -1
        else:
            return 1
