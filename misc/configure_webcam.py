#!/usr/bin/env python

from subprocess import call


if __name__ == "__main__":
    device = '/dev/video1'

    # Turn off auto-exposure.
    # For our webcam, exposure_auto=1 seems to correspond to 'manual', and
    # exposure_auto=3 (the default) seems to correspond to 'aperture priority'
    # (according to the controls guvcview shows).
    call(['v4l2-ctl', '-d', device, '-c', 'exposure_auto=1'])

    # Take some images at various exposures.
    # The range is 3 - 2047 according to `v4l2-ctl -d /dev/video1 -l`.
    exposure_range = range(100, 2001, 100)
    for exposure in exposure_range:
        print exposure
        call(['v4l2-ctl', '-d', device,
                '-c', 'exposure_absolute={}'.format(exposure)])
        call(['streamer', '-c', device, '-s', '640x480',
                '-o', '{:04d}.jpeg'.format(exposure)])
    print "Wrote images to current dir."
