#!/usr/bin/env python

import rospy
from simple_script_server import *
sss = simple_script_server()

import os
import sys
from optparse import OptionParser

class MoveComponent():
    def __init__(self, options):
        rospy.init_node('component_test', anonymous=True)
        self.component = str(options.can_device)+"/"+str(options.component)
        self.reps = int(options.repetitions)
        self.default_vel = float(options.default_vel)
 
    def run_test(self):
        rospy.set_param("/script_server/"+self.component+"/default_vel", self.default_vel)
        poses = rospy.get_param("/script_server/" +self.component+"/test")
        for i in range(0, self.reps):
            rospy.loginfo(">>>> Executing iteration %s of %s", str(i), str(self.reps))
            for key, value in poses.iteritems():
                rospy.loginfo("Moving to pose %s", str(key))
                handle = sss.move(self.component, value, True)
                if (handle.get_error_code() == 4):
                    rospy.logerr("Could not move to %s. Aborting!", key)
                    break
                else:
                    rospy.loginfo("Moved to pose %s successfully", key)
        handle = sss.move(self.component, "home", True)
        rospy.loginfo(">>>> Test finished")

    def component_init(self):
        # call init
        rospy.loginfo("Initializing component")
        while not rospy.is_shutdown():
            handle = sss.init(self.component)
            if not (handle.get_error_code() == 0):
                rospy.logerr("Could not initialize %s. Retrying...", self.component)
                rospy.sleep(5.0)
            else:
                rospy.loginfo("Component %s initialized successfully", self.component)
                break


if __name__ == '__main__':
    _usage = """%prog [options]
    type %prog -h for more info."""
    
    parser = OptionParser(usage=_usage, prog=os.path.basename(sys.argv[0]))
    parser.add_option(
        '-c', '--component', dest='component',
        help="Component that is going to be tested")
    parser.add_option(
        '-d', '--can_device', dest='can_device',
        help="CAN device the component is connected to")
    parser.add_option(
        '-r', '--reps', dest='repetitions', default=5,
        help="Number of repetitions for each test cycle")
    parser.add_option(
        '-v', '--default_vel', dest='default_vel', default=0.2,
        help="Overwrite default velocity of component")
    (options, args) = parser.parse_args()
    if not any([options.component]):
        parser.print_usage()
        quit()
    try:
        component = MoveComponent(options)
        component.component_init()
        component.run_test()
    except rospy.ROSInterruptException:
        pass
