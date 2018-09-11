cob_devices_test
=================


## BASE:
- Connect the USB-Hub (for laser scanner, light, joystick) and CAN-Dongle (for bms) to the computer via two separate USB cables. 

Setup udev rules

```
su robot
sudo /home/support/git/robot_ws/src/cob_hardware_test/cob_devices_test/scripts/udev_joy.sh
sudo /home/support/git/robot_ws/src/cob_hardware_test/cob_devices_test/scripts/udev_s300.sh
```

Start launchfile in a new window

```roslaunch cob_devices_test base.launch flexisoft_host:=XX.XX.XX.99```

### Light:
The LEDs of the wheel covers will light up one-by-one (CIRCLE_COLOR) red-green-blue.
Check all wheel-colors.

### Laserscanner:
Select **Base_Test** in RVIZ, the laserscanner points should be visible in RVIZ (front-green, left-white, right-yellow).
Check the correct assignment.

### BMS:
Check diagnostics in the `rqt_robot_monitor`

### Joystick:
Check diagnostics in the `rqt_robot_monitor` and/or execute `rostopic echo /joy` and use the joystick.


## TORSO:
- Connect the light to the computer

Start launchfile

```roslaunch cob_devices_test torso.launch```

### Light:
The LEDs of the LED ring will light up one-by-one (CIRCLE_COLOR) red-green-blue.


## HEAD:
- Connect the head to the power supply (first, connect before switching on the supply).
- Connect the computer to the head display
- Connect the camera, touchscreen and soundcard to the computer.

Start launchfile

```roslaunch cob_devices_test head.launch```

### Camera:
Select **Head_Test** in RVIZ, the Camera Image of the head camera should be visible

### Sound: 
Select the correct Input and Output devices **(Sound Blaster Play! 2)** at the Sound Settings (set Input Volume to 100%)
and record sound with 

```arecord -C test.wav```

See if sound has been recorded successfully with

```aplay test.wav```

Test Speakers with **Test Speakers** within the Sound Settings (check if left and right is correct

### Touchscreen:
Touch the display of the robot
