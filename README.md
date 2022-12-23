# robocar\_ws

## Teleop Notes

To control the robocar, I used an Xbox One S Controller with bluetooth. On Ubuntu, you need to disable ertm for Xbox controllers to work, which I did by following [TCIII's forum post](https://forums.developer.nvidia.com/t/disabling-ertm-permanently-in-jetpack-4-4-ubuntu-18-04-on-nano-4gb/159567).

First, add yourself to the `bluetooth` group with `sudo adduser $USER bluetooth`.

You can connect to the Xbox controller by first pressing the "pair" button on the controller until the Xbox light starts flashing quickly, and then running `bluetoothctl` on the Nano. While in the bluetoothctl shell, run `scan on` until you see the controller, then turn off the scanning with `scan off`. Copy the MAC address and connect with `pair XX:XX:XX:XX:XX:XX` and then `connect XX:XX:XX:XX:XX:XX`.

To auto-connect to the controller, you will need to trust it by running `trust XX:XX:XX:XX:XX:XX`. The controller should be connected now (solid Xbox light), so you can exit the bluetoothctl shell with `exit`.

Finally, to give the user access to input devices in `/dev/input/` like the controller, add a udev rule:
```bash
echo "KERNEL==\"event*\", NAME=\"input/%k\", MODE=\"666\", GROUP=\"input\"" | sudo tee -a /etc/udev/rules.d/99-input.rules
```

