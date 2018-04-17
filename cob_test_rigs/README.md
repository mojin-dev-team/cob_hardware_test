Start bringup
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


Komponenten:
COMPONENT: one of torso2,torso3,head2,...(NO FDM)

﻿Start bringup
roslaunch cob_test_rigs COMPONENT.launch [can_device:=can1]

initialize:
rosservice call /COMPONENT/driver/[init/recover]

Testscript
rosrun cob_test_rigs test_components.py -c COMPONENT -r 1

Options:
  -c COMPONENT, --component=COMPONENT
                        Component that is going to be tested
  -r REPETITIONS, --reps=REPETITIONS
                        Number of repetitions for each test cycle

start cob_console
rosrun cob_script_server cob_console 

use in cob_console
sss.move("COMPONENT","CONFIG")
sss.init("COMPONENT")

exit cob_console
STRG + D, then ENTER

get available configs/poses
rosparam get /script_server/torso2/


==========================================================================
SINGLE JOINT TESTING

roslaunch cob_test_rigs single_[elmo/schunk].launch can_device:=can0 can_id:=33

rosservice call /single_elmo/driver/init 

start graphical tools
rqt

start controller




move single joint

