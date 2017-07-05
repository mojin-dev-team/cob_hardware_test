#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32


class FDM():

    def __init__(self):
        rospy.init_node('fdm_test')
        rospy.Timer(rospy.Duration(0.1), self.pub_vel)
        self.pub_steer = rospy.Publisher('/base_steer', Float32, queue_size=1)
        self.pub_drive = rospy.Publisher('/base_drive', Float32, queue_size=1)
        self.vel_steer = 0.0
        self.vel_drive = 0.0

    def run_test(self):
        #set_vel(vel_steer, vel_drive)
        self.set_vel(0.5, 0.5)
        rospy.sleep(3)
        self.set_vel(-0.5, -0.5)
        rospy.sleep(3)
        self.set_vel(0.0, 0.0)

    def set_vel(self, vel_steer, vel_drive):
        self.vel_steer = vel_steer
        self.vel_drive = vel_drive

    def pub_vel(self, event):
        self.pub_steer.publish(self.vel_steer)
        self.pub_drive.publish(self.vel_drive)

if __name__ == '__main__':
    try:
        fdm = FDM()
        fdm.run_test()
    except rospy.ROSInterruptException:
        pass
