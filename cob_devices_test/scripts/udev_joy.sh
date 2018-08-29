#!/bin/bash

sudo rm -rf /tmp/js*
sudo rm -rf /dev/joypad

## Joystick ##
JoyAttr1='ATTRS{idVendor}=="046d"'
#JoyAttr2='ATTRS{idProduct}=="c21f"'

sudo chmod 666 /dev/input/js0
sudo udevadm info -a -p $(udevadm info -q path -n /dev/input/js0) > /tmp/js0
if grep -qs $JoyAttr1 /tmp/js0
then
    sudo ln -s input/js0 /dev/joypad
fi

sudo chmod 666 /dev/input/js1
sudo udevadm info -a -p $(udevadm info -q path -n /dev/input/js1) > /tmp/js1
if grep -qs $JoyAttr1 /tmp/js1
then
    sudo ln -s input/js1 /dev/joypad
fi

sudo chmod 666 /dev/input/js2
sudo udevadm info -a -p $(udevadm info -q path -n /dev/input/js2) > /tmp/js2
if grep -qs $JoyAttr1 /tmp/js2
then
    sudo ln -s input/js2 /dev/joypad
fi
