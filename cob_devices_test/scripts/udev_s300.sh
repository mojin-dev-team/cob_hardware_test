#!/bin/bash

sudo rm -rf /tmp/usb*
sudo rm -rf /dev/ttyScanFront
sudo rm -rf /dev/ttyScanLeft
sudo rm -rf /dev/ttyScanRight

for file in /dev/ttyUSB*; do
  sudo chmod 666 $file
  sudo rm -f /tmp/usb${file: -1}
  sudo udevadm info -a -p $(udevadm info -q path -n $file) > /tmp/usb${file: -1}
  sudo chmod 666 /tmp/usb${file: -1}
done

results=()
count=0

for file in /tmp/usb*; do
  if grep --quiet 'ATTRS{serial}=="F' $file; then
    result=$(ls -l |grep -R 'ATTRS{serial}=="F' $file)
    echo "found scanner with $result"
    results[$count]=$result
    count=$((count+1))
  fi
done

echo "found $count scanners: $results"

if [[ ${results[0]} == ${results[1]} ]]; then
  ATTRSSerialFL=${results[0]}
  ATTRSSerialR=${results[2]}
elif [[ ${results[1]} == ${results[2]} ]]; then
  ATTRSSerialFL=${results[1]}
  ATTRSSerialR=${results[0]}
elif [[ ${results[0]} == ${results[2]} ]]; then
  ATTRSSerialFL=${results[0]}
  ATTRSSerialR=${results[1]}
fi

ATTRSSerialFL="$( echo "$ATTRSSerialFL" | sed 's/ //g' )"
ATTRSSerialR="$( echo "$ATTRSSerialR" | sed 's/ //g' )"

echo $ATTRSSerialFL
echo $ATTRSSerialR

## ScanFront ##
ScanFrontAttr1='ATTRS{bInterfaceNumber}=="00"'
ScanFrontAttr2=$ATTRSSerialFL

## ScanLeft ##
ScanLeftAttr1='ATTRS{bInterfaceNumber}=="01"'
ScanLeftAttr2=$ATTRSSerialFL

## ScanRight ##
ScanRightAttr1='ATTRS{bInterfaceNumber}=="00"'
ScanRightAttr2=$ATTRSSerialR


sudo chmod 666 /dev/ttyUSB0
sudo udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB0) > /tmp/usb0
if grep -qs $ScanFrontAttr1 /tmp/usb0  && grep -qs $ScanFrontAttr2 /tmp/usb0
then
    sudo ln -s ttyUSB0 /dev/ttyScanFront
fi
if grep -qs $ScanLeftAttr1 /tmp/usb0  && grep -qs $ScanLeftAttr2 /tmp/usb0
then
    sudo ln -s ttyUSB0 /dev/ttyScanLeft
fi
if grep -qs $ScanRightAttr1 /tmp/usb0  && grep -qs $ScanRightAttr2 /tmp/usb0
then
    sudo ln -s ttyUSB0 /dev/ttyScanRight
fi

sudo chmod 666 /dev/ttyUSB1
sudo udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB1) > /tmp/usb1
if grep -qs $ScanFrontAttr1 /tmp/usb1  && grep -qs $ScanFrontAttr2 /tmp/usb1
then
    sudo ln -s ttyUSB1 /dev/ttyScanFront
fi
if grep -qs $ScanLeftAttr1 /tmp/usb1  && grep -qs $ScanLeftAttr2 /tmp/usb1
then
    sudo ln -s ttyUSB1 /dev/ttyScanLeft
fi
if grep -qs $ScanRightAttr1 /tmp/usb1  && grep -qs $ScanRightAttr2 /tmp/usb1
then
    sudo ln -s ttyUSB1 /dev/ttyScanRight
fi

sudo chmod 666 /dev/ttyUSB2
sudo udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB2) > /tmp/usb2
if grep -qs $ScanFrontAttr1 /tmp/usb2  && grep -qs $ScanFrontAttr2 /tmp/usb2
then
    sudo ln -s ttyUSB2 /dev/ttyScanFront
fi
if grep -qs $ScanLeftAttr1 /tmp/usb2  && grep -qs $ScanLeftAttr2 /tmp/usb2
then
    sudo ln -s ttyUSB2 /dev/ttyScanLeft
fi
if grep -qs $ScanRightAttr1 /tmp/usb2  && grep -qs $ScanRightAttr2 /tmp/usb2
then
    sudo ln -s ttyUSB2 /dev/ttyScanRight
fi

sudo chmod 666 /dev/ttyUSB3
sudo udevadm info -a -p $(udevadm info -q path -n /dev/ttyUSB3) > /tmp/usb3
if grep -qs $ScanFrontAttr1 /tmp/usb3 && grep -qs $ScanFrontAttr2 /tmp/usb3
then
    sudo ln -s ttyUSB3 /dev/ttyScanFront
fi
if grep -qs $ScanLeftAttr1 /tmp/usb3  && grep -qs $ScanLeftAttr2 /tmp/usb3
then
    sudo ln -s ttyUSB3 /dev/ttyScanLeft
fi
if grep -qs $ScanRightAttr1 /tmp/usb3  && grep -qs $ScanRightAttr2 /tmp/usb3
then
    sudo ln -s ttyUSB3 /dev/ttyScanRight
fi
