cob_devices_test
=================

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

Test Speakers with **Test Speakers** within the Sound Settings (check if left and right is correct)
### Touchscreen:
Touch the display of the robot

## SENSORRING:
- Connect the camera to the computer

Start launchfile

```roslaunch cob_devices_test sensorring.launch```

### Camera:
Select **Sensorring_Test** in RVIZ, the Camera Image of the sensorring camera should be visible.
Also the Pointclouds should be visible in RVIZ

## TORSO:
- Connect the cameras and the light to the computer

Start launchfile

```roslaunch cob_devices_test torso.launch```

### Light:
Torso LED ring will flash (in that order) white-red-green-blue

### Camera:
Select **Torso_Test** in RVIZ, the Camera Images of the torso cameras should be visible.
Also the Pointclouds should be visible in RVIZ


## BASE:
- Connect the laserscanners and the light to the computer

Start launchfile

```roslaunch cob_devices_test base.launch```

### Light:
Base wheel-cover-LEDs will flash (in that order) white-red-green-blue

### Laserscanner:
Select **Base_Test** in RVIZ, the laserscanner points should be visible in RVIZ (front-green, left-white, right-yellow)
