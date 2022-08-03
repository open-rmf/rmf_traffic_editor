import math
import numpy as np


class Transform:
    """This class represents an 2D rotation, scaling, and translation."""

    def __init__(self):
        self.set_rotation(0.0)
        self.set_translation(0.0, 0.0)
        self.set_scale(1.0)
        self.x = 0
        self.y = 0

    def set_rotation(self, rotation):
        """Calculate rotation matrix"""
        self.rotation = rotation
        cr = math.cos(self.rotation)
        sr = math.sin(self.rotation)
        self.rot_mat = np.array([[cr, -sr], [sr, cr]])

    def set_translation(self, tx, ty):
        self.translation = (tx, ty)
        self.t_vec = np.array([[self.translation[0]], [self.translation[1]]])

    def set_scale(self, scale):
        self.scale = scale

    def transform_point(self, p):
        vec = np.array([[p[0]], [p[1]]])
        transformed = (self.rot_mat @ vec) * self.scale + self.t_vec
        return (transformed[0].item(), transformed[1].item())

    def set_from_fiducials(self, fiducial_pairs, ref_scale):
        print(f'Transform.set_from_fiducials()')
        print(f'  {len(fiducial_pairs)} fiducial pairs:')
        for f_pair in fiducial_pairs:
            f0 = f_pair[0]
            f1 = f_pair[1]
            print(
                f'    ({float(f0.x):.5}, {float(f0.y):.5})'
                f' -> ({float(f1.x):.5}, {float(f1.y):.5})'
                f'   {f0.name}')

        # calculate the bearings and distances between each fiducial pair
        distances = []
        bearings = []
        for f0_idx in range(0, len(fiducial_pairs)):
            for f1_idx in range(f0_idx + 1, len(fiducial_pairs)):
                f0_pair = fiducial_pairs[f0_idx]
                f1_pair = fiducial_pairs[f1_idx]
                print(f'    calc dist {f0_pair[0].name} <=> {f1_pair[0].name}')

                ref_bearing = f0_pair[0].bearing(f1_pair[0])
                target_bearing = f0_pair[1].bearing(f1_pair[1])
                bearings.append((ref_bearing, target_bearing))

                ref_dist = f0_pair[0].distance(f1_pair[0])
                target_dist = f0_pair[1].distance(f1_pair[1])
                distances.append((ref_dist, target_dist))

        print("Bearings:")
        print(bearings)
        if len(bearings) == 0:
            return

        # compute the circular mean of the difference between the bearings
        bearing_sum = [0.0, 0.0]
        for bearing_pair in bearings:
            d_theta = bearing_pair[1] - bearing_pair[0]
            bearing_sum[0] += math.sin(d_theta)
            bearing_sum[1] += math.cos(d_theta)
            print(f'  {d_theta}')
        mean_bearing_difference = -math.atan2(bearing_sum[0], bearing_sum[1])
        print(f'  Circular mean: {mean_bearing_difference}')
        self.set_rotation(mean_bearing_difference)

        print("Distances:")
        print(distances)

        mean_rel_scale = 0.0
        for distance in distances:
            mean_rel_scale += distance[1] / distance[0]
        mean_rel_scale /= float(len(distances))
        print(f'mean relative scale: {mean_rel_scale}')
        self.set_scale(ref_scale / mean_rel_scale)

        mean_translation = [0.0, 0.0]
        cr = math.cos(mean_bearing_difference)
        sr = math.sin(mean_bearing_difference)
        for f_pair in fiducial_pairs:
            rot_mat = np.array([[cr, -sr], [sr, cr]])
            f1x = f_pair[1].x / mean_rel_scale
            f1y = f_pair[1].y / mean_rel_scale
            rot_f_pair1 = rot_mat @ np.array([[f1x], [f1y]])
            rot_f1x = rot_f_pair1[0].item()
            rot_f1y = rot_f_pair1[1].item()

            mean_translation[0] += rot_f1x - f_pair[0].x
            mean_translation[1] += rot_f1y - f_pair[0].y

        if len(fiducial_pairs):
            mean_translation[0] *= -ref_scale / float(len(fiducial_pairs))
            mean_translation[1] *= -ref_scale / float(len(fiducial_pairs))
        print(
            f'translation: '
            f'({mean_translation[0]:.5}, '
            f'{mean_translation[1]:.5})')
        self.set_translation(*mean_translation)
