# probe-rs udev rules fix

`probe-rs list` finds the device, but `probe-rs run` fails with `Error: Probe not found`.

The probe-rs udev rules use deprecated `GROUP="plugdev"`.
Updating systemd changed behavior, non-existent group prevents `TAG+="uaccess"` from being applied.

```bash
probe-rs list

lsusb -d 303a:1001
ls -la /dev/bus/usb/BUS/DEVICE
getfacl /dev/bus/usb/BUS/DEVICE
```

```bash
sudo sed -i 's/, GROUP="plugdev"//g' /etc/udev/rules.d/69-probe-rs.rules
sudo udevadm control --reload-rules
```

Reconnect device.

See also:

- <https://wiki.archlinux.org/title/Udev#Allowing_regular_users_to_use_devices>
- <https://github.com/probe-rs/probe-rs/issues/3566>
