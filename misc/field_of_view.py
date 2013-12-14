#!/usr/bin/env python

import numpy as np


def dist_on_ground(fov, height):
    return 2 * height * np.tan(fov / 2)


if __name__ == "__main__":
    fov_long_side = 69 * np.pi / 180
    fov_short_side = 42 * np.pi / 180

    heights = [1, 2, 4, 8, 16]
    for height in heights:
        ground_long_side = dist_on_ground(fov_long_side, height)
        ground_short_side = dist_on_ground(fov_short_side, height)
        print "at height {} meters, see an area of {} by {} meters".format(
            height, ground_long_side, ground_short_side)
    print

    heights = [0.88, 1.20, 1.70, 2.26]
    for height in heights:
        ground_long_side = dist_on_ground(fov_long_side, height)
        ground_short_side = dist_on_ground(fov_short_side, height)
        pixel_long_mm = ground_long_side * 1000.0 / 640
        pixel_short_mm = ground_short_side * 1000.0 / 480
        print "at height {} meters, a pixel is {} by {} mm on the ground".format(
            height, pixel_long_mm, pixel_short_mm)
