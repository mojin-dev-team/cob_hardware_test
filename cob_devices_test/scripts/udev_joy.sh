#!/bin/bash

sudo rm -rf /tmp/js*
sudo rm -rf /dev/joypad

## Joystick ##
JoyAttr1='ATTRS{idVendor}=="046d"'
#JoyAttr2='ATTRS{idProduct}=="c21f"'

for file in /dev/input/js*; do
    #echo $file
    sudo chmod 666 $file 2> /dev/null
    sudo udevadm info -a -p $(udevadm info -q path -n $file) > /tmp/js${file: -1} 2> /dev/null
    if grep -qs $JoyAttr1 /tmp/js${file: -1}
    then
        sudo ln -s input/js${file: -1} /dev/joypad
    fi
done

if [ -L /dev/joypad ] && [ -e /dev/joypad ] ; then
    echo "Joypad found"
else
    echo "Joypad not found"
fi
