#!/bin/bash

sudo rm -rf /tmp/usb*
sudo rm -rf /dev/ttyScanFront
sudo rm -rf /dev/ttyScanLeft
sudo rm -rf /dev/ttyScanRight

for file in /dev/ttyUSB*; do
  #echo $file
  sudo chmod 666 $file 2> /dev/null
  sudo rm -f /tmp/usb${file: -1}
  sudo udevadm info -a -p $(udevadm info -q path -n $file) > /tmp/usb${file: -1} 2> /dev/null
  sudo chmod 666 /tmp/usb${file: -1} 2> /dev/null
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

echo -e "found $count scanners\n"

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

## ScanFront ##
ScanFrontAttr1='ATTRS{bInterfaceNumber}=="00"'
ScanFrontAttr2=$ATTRSSerialFL

## ScanLeft ##
ScanLeftAttr1='ATTRS{bInterfaceNumber}=="01"'
ScanLeftAttr2=$ATTRSSerialFL

## ScanRight ##
ScanRightAttr1='ATTRS{bInterfaceNumber}=="00"'
ScanRightAttr2=$ATTRSSerialR


for file in /dev/ttyUSB*; do
    sudo chmod 666 $file 2> /dev/null
    sudo udevadm info -a -p $(udevadm info -q path -n $file) > /tmp/usb${file: -1} 2> /dev/null
    if grep -qs $ScanFrontAttr1 /tmp/usb${file: -1}  && grep -qs $ScanFrontAttr2 /tmp/usb${file: -1}
    then
        sudo ln -s ttyUSB${file: -1} /dev/ttyScanFront
    fi
    if grep -qs $ScanLeftAttr1 /tmp/usb${file: -1}  && grep -qs $ScanLeftAttr2 /tmp/usb${file: -1}
    then
        sudo ln -s ttyUSB${file: -1} /dev/ttyScanLeft
    fi
    if grep -qs $ScanRightAttr1 /tmp/usb${file: -1}  && grep -qs $ScanRightAttr2 /tmp/usb${file: -1}
    then
        sudo ln -s ttyUSB${file: -1} /dev/ttyScanRight
    fi
done

if [ -L /dev/ttyScanFront ] && [ -e /dev/ttyScanFront ] ; then
    echo "ScanFront found"
else
    echo "ScanFront not found"
fi
if [ -L /dev/ttyScanLeft ] && [ -e /dev/ttyScanLeft ] ; then
    echo "ScanLeft found"
else
    echo "ScanLeft not found"
fi
if [ -L /dev/ttyScanRight ] && [ -e /dev/ttyScanRight ] ; then
    echo "ScanRight found"
else
    echo "ScanRight not found"
fi

