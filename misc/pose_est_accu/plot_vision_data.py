#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt


def parse_data(f):
    """
    Parse TestPoseFromCamera data and return xs, ys, zs, yaws.
    """
    data = []
    for line in f:
        line = line.strip()
        if not line:
            continue
        assert line[0] == '[' and line[-1] == ']'
        parts = line[1:-1].split('; ')
        assert len(parts) == 4
        parts = map(float, parts)
        data.append(parts)
    data = np.array(data)
    return data[:, 0], data[:, 1], data[:, 2], data[:, 3]


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print >>sys.stderr, "Usage: {} data-file".format(sys.argv[0])
        sys.exit(1)
    xs, ys, zs, yaws = parse_data(open(sys.argv[1]))
    yaws_deg = yaws * 180 / np.pi
    print "x  : mean={} std={}".format(np.mean(xs), np.std(xs))
    print "y  : mean={} std={}".format(np.mean(ys), np.std(ys))
    print "z  : mean={} std={}".format(np.mean(zs), np.std(zs))
    print "yaw: mean={} std={}".format(np.mean(yaws), np.std(yaws))
    print "yaw_deg: mean={} std={}".format(np.mean(yaws_deg), np.std(yaws_deg))
