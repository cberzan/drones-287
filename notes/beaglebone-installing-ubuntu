## Initial installation:

On laptop:

1. Downloaded 12.04.3 image from [here][1].
1. `xz -cd ubuntu-precise-12.04.3-armhf-3.8.13-bone28.img.xz >/dev/mmcblk0`
   to get the image onto a microSD card.
1. Also copied the xz image onto the rootfs partition on the microSD card.

On the beaglebone-black:

1. Verified that the beaglebone boots angstrom off its internal emmc. (The
   microSD card had to be removed for this.)
1. Plugged the microSD in and rebooted the beaglebone. Allowed the "waiting for
   network configuration" message to time out. It dropped me to a login prompt
   (ubuntu/ubuntu).
1. `xz -cd ubuntu-precise-12.04.3-armhf-3.8.13-bone28.img.xz >/dev/mmcblk1`
   to get the ubuntu image onto the beaglebone's internal emmc.
1. Powered off, removed microSD card, booted the beaglebone again. Allowed the
   DHCP message to time out again.
1. Plugged the wifi adapter plugged into the USB hub. Set up my home wifi
   network credentials in `/etc/network/interfaces`. Tried `ifup wlan0`. It
   took a long time, but eventually it got me online. Woot.
1. At this point I can ssh to the beaglebone's IP (as user ubuntu).
1. `apt-get update` and installed a bunch of useful stuff (`vim-nox byobu htop`)
1. Commented out eth0 from `/etc/network/interfaces` to get rid of the DHCP
   timeout at boot. Didn't help. Followed [this][2] to get rid of the DHCP
   timeout properly (edit `/etc/init/failsafe.conf` and remove sleep commands).
1. Rebooted beaglebone; it boots without waiting for DHCP now, but it takes a
   while to acquire an IP address over wifi.


## Fixed problems:

- Install missing basics: `psmisc man-db`
- Adjust font to something readable:
  ```
  sudo setfont /usr/share/consolefonts/Lat7-Terminus32x16.psf.gz
  ```
  Or automatically at boot: `sudo dpkg-reconfigure console-setup`
- Change timezone to Pacific: `sudo dpkg-reconfigure tzdata`
- When the beaglebone cold-boots, the time is reset to 2000.
  This causes some weirdness to spawn a bunch of
  `apt-get -s -o Debug::NoLocking=true upgrade` processes, clogging the system.
  Turns out this is byobu's fault. See [this bug][3] and [this fix][4], which
  didn't make it into ubuntu 12.04 yet.
  How to work around it: edit `/usr/lib/byobu/updates_available` and add
  neutralize the function `___update_cache` by making it `return` right away.


## Remaining problems:

- Wifi is unreliable; stops working after a while. Nothing in the logs.
- USB hub with wifi & keyboard in it is unreliable. Not enough power?



[1]: http://www.armhf.com/index.php/boards/beaglebone-black/#precise
[2]: http://tech.pedersen-live.com/2012/05/disable-waiting-for-network-configuration-messages-on-ubuntu-boot/
[3]: https://bugs.launchpad.net/byobu/+bug/999151
[4]: https://github.com/dustinkirkland/byobu/commit/64d8ac8f459d145993e0840eeade4835df3e6858
