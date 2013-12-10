#!/usr/bin/env python

import numpy as np

def dist_on_ground(fov, height):
    return 2 * height * np.tan(fov / 2)

if __name__ == "__main__":
    fov_long_side = 69 * np.pi / 180
    fov_short_side = 42 * np.pi / 180
    heights = [1, 2, 4, 8, 16]
    for height in heights:
        print "at height {} meters, see an area of {} by {} meters".format(
            height,
            dist_on_ground(fov_long_side, height),
            dist_on_ground(fov_short_side, height))

