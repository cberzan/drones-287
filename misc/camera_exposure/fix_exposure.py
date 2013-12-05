#!/usr/bin/env python

from subprocess import call
import sys


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print >>sys.stderr, "Usage: {} device exposure".format(sys.argv[0])
        sys.exit(1)
    device = sys.argv[1]
    exposure = sys.argv[2]

    # Turn off auto-exposure.
    call(['v4l2-ctl', '-d', device, '-c', 'exposure_auto=1'])

    # Set desired manual exposure.
    call(['v4l2-ctl', '-d', device,
            '-c', 'exposure_absolute={}'.format(exposure)])
    print "Done."
