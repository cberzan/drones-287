#!/usr/bin/env python

import serial
import time


if __name__ == "__main__":
    port = serial.Serial(
        '/dev/ttyACM0', 115200, timeout=0,
        dsrdtr=False, rtscts=False, xonxoff=False)

    timeout_sec = 15
    sleep_sec = 0.1
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        n = port.inWaiting()
        if n > 0:
            s = port.read(n)
            print "hex: {}".format(s.encode('hex'))
        else:
            print '.'
        time.sleep(sleep_sec)

    port.close()
