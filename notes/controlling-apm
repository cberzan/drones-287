## Experimentation:

Run roscore in one terminal.

Connect the APM, and run roscopter in another terminal.

```
cd ~/catkin_ws/src/roscopter
nodes/roscopter.py --device=/dev/ttyACM0 --baudrate=115200 --enable-control=true
```

In a third terminal, watch the RC topic:

```
rostopic echo /rc
```

Assuming the radio controller is connected, you will see lines like:

```
channel: [1492, 1488, 1489, 1487, 1888, 1498, 1499, 1498]
```

See [here][1] for the meaning of the channels. Roughly:

1. roll (right thumb left/right)
2. pitch (right thumb up/down)
3. throttle (left thumb up/down)
4. yaw (left thumb left/right)
5. mode switch (some switch, e.g. gear or flap)

The values on these channels are between 1000 and 2000. The mode-switch
channel, if connected to a 3-way switch on the remote, takes values ~1100,
~1500, ~1900. For more about flight modes, see [here][2].

Now we can control these using roscopter. In a fourth terminal:

```
rostopic pub /send_rc roscopter/RC -- '[1000, 1000, 1000, 1000, 0, 0, 0, 0]'
```

This will fix the first four channels to 1000. Note that even after hitting
Ctrl+C, the override values are still in power. To give control back to the
remote, send zero values (documented [here][3]):

```
rostopic pub /send_rc roscopter/RC -- '[0, 0, 0, 0, 0, 0, 0, 0]'
```


## How to give control to the remote in emergencies

The ROS node sending controls to `/send_rc` should watch channel 5 of `/rc`. If
that takes a pre-specified value, the node should immediately send all zeros to
`/send_rc`. This means that as soon as the mode switch is flipped on the
remote, all control is given back to the remote.


[1]: http://copter.ardupilot.com/wiki/connecting-your-rc-input-and-motors/
[2]: https://code.google.com/p/arducopter/wiki/AC2_ModeSwitch
[3]: https://pixhawk.ethz.ch/mavlink/#RC_CHANNELS_OVERRIDE
