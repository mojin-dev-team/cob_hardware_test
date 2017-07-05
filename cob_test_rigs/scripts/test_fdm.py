#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger


class FDM():

    def __init__(self):
        rospy.init_node('fdm_test')
        rospy.Timer(rospy.Duration(0.1), self.pub_vel)
        self.pub = rospy.Publisher('/base/joint_group_velocity_controller/command', Float64MultiArray, queue_size=1)
        self.vel_drive = 0.0
        self.vel_steer = 0.0
        base_init = rospy.ServiceProxy('/base/driver/init', Trigger)
        rospy.wait_for_service('/base/driver/init')
        base_init()

    def run_test(self):
        #set_vel(vel_steer, vel_drive)
        steps = 5
        max_vel_steer = 50.0#94.0
        max_vel_drive = 0.0#20.0
        for i in range(1, steps + 1):
            if rospy.is_shutdown():
                return
            value_steer = i/float(steps)*max_vel_steer
            value_drive = i/float(steps)*max_vel_drive
            print "value_steer:", value_steer
            print "value_drive:", value_drive
            self.set_vel(value_steer ,value_drive)
            rospy.sleep(3)

        self.set_vel(0.0, 0.0)
        rospy.sleep(3)

        for i in range(1, steps + 1):
            if rospy.is_shutdown():
                return
            value_steer = i/float(steps)*max_vel_steer
            value_drive = i/float(steps)*max_vel_drive
            print "value_steer:", value_steer
            print "value_drive:", value_drive
            self.set_vel(value_steer, value_drive)
            rospy.sleep(3)
            self.set_vel(-value_steer, -value_drive)
            rospy.sleep(3)
            self.set_vel(0.0, 0.0)
            rospy.sleep(3)
            for j in range(1,10):
                self.set_vel(value_steer, value_drive)
                rospy.sleep(1)
                self.set_vel(-value_steer, -value_drive)
                rospy.sleep(1)
            self.set_vel(0.0, 0.0)
            rospy.sleep(1)

    def set_vel(self, vel_steer, vel_drive):
        self.vel_steer = vel_steer
        self.vel_drive = vel_drive

    def pub_vel(self, event):
	vel = Float64MultiArray()
	vel.data = [self.vel_steer, self.vel_drive]
        self.pub.publish(vel)

if __name__ == '__main__':
    try:
        fdm = FDM()
        fdm.run_test()
    except rospy.ROSInterruptException:
        pass
