FDM

﻿Start bringup
roslaunch cob_test_rigs fdm.launch can_id_steer:=3 can_id_drive:=4

Testscript
rosrun cob_test_rigs test_fdm.py

--------------------------------------------
Für canopen elmo console:
cd ~/git/canopen_test_utils
source devel/setup.bash 
rosrun canopen_test_utils canopen_elmo_console <can-device> <can-ID>
#Bsp.: rosrun canopen_test_utils canopen_elmo_console can0 1

Sonstiges:
rosservice call /fdm/driver/recover

--------------------------------------------
ROS MASTER URI setzen: 
export ROS_MASTER_URI=http://10.4.ROBOTER-Nr.11:11311

ifconfig = eigene IP Adresse aus eth0
export ROS_IP=eigene IP Adresse 


=========================================================================
Komponenten

CAN_DEVICE: e.g. can0, can1,...
COMPONENT: e.g. torso2, torso3, head2,...(NO FDM)

﻿Start bringup
roslaunch cob_test_rigs COMPONENT.launch [can_device:=can1]

initialize:
rosservice call /CAN_DEVICE/COMPONENT/driver/[init/recover]

Testscript
rosrun cob_test_rigs test_components.py -c COMPONENT -d CAN_DEVICE -r 1 -v 0.4

Options:
  -h, --help            show this help message and exit
  -c COMPONENT, --component=COMPONENT
                        Component that is going to be tested
  -d CAN_DEVICE, --can_device=CAN_DEVICE
                        CAN device the component is connected to
  -r REPETITIONS, --reps=REPETITIONS
                        Number of repetitions for each test cycle
  -v DEFAULT_VEL, --default_vel=DEFAULT_VEL
                        Overwrite default velocity of component

start cob_console
rosrun cob_script_server cob_console 

use in cob_console
sss.move("CAN_DEVICE/COMPONENT","test/CONFIG")
sss.init("CAN_DEVICE/COMPONENT")

exit cob_console
STRG + D, then ENTER

get available configs/poses
rosparam get /script_server/CAN_DEVICE/COMPONENT


==========================================================================
SINGLE JOINT TESTING

roslaunch cob_test_rigs single_[elmo/schunk].launch can_device:=can0 can_id:=XX

can_ids:
 - torso: 31, 32, 33
 - head: 70, 71, 72
 - sensorring: 73
 - arm: 61, 62, 63, 64, 65, 66, 67

rosservice call /single_[elmo/schunk]/driver/init 

start graphical tools
rqt

start controller (in rqt window)
go to tab controller manager
add joint_trajectory_controller from drop-down menu
right click on controller name -> press start (needs to be "running" afterwards)

move single joint
go to tab joint trajectory controller
select joint_trajectory_controller from drop down
press red button to activate slider (button turns green)
move the slider or enter desired joint position [rad] directly

