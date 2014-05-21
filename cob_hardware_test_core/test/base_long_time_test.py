#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test_core')

import sys
import unittest
import rospy
import rostest

from std_msgs.msg import String
from simple_script_server import *
from pr2_controllers_msgs.msg import *
from dialog_client import *


class HardwareTest(unittest.TestCase):
    def __init__(self, *args):

        super(HardwareTest, self).__init__(*args)
        rospy.init_node('base_long_time_test')
        self.message_received = False
        self.sss = simple_script_server()

    def test_base(self):
        self.sss.init("base")
        self.assertTrue(dialog_client(0, 'Ready to start the base test?'))

        tests_count = 0
        while not rospy.is_shutdown() and tests_count <= 0:

	    ###
	    # A dialog to ask the user whether he wants to terminate the test
	    ###

            handle = self.sss.move("base","test_0")
            self.assertEqual(handle.get_state(), 3)
            handle.wait()
            
            handle = self.sss.move("base","test_1")
            self.assertEqual(handle.get_state(), 3)
            handle.wait()
            
            handle = self.sss.move("base","test_2")
            self.assertEqual(handle.get_state(), 3)
            handle.wait()
            
            handle = self.sss.move("base","test_3")
            self.assertEqual(handle.get_state(), 3)
            handle.wait()
            
            handle = self.sss.move("base","test_0")
            self.assertEqual(handle.get_state(), 3)
            handle.wait()

            tests_count += 1

if __name__ == '__main__':
    # Copied from hztest: A dirty hack to work around an apparent race condition at startup
    # that causes some hztests to fail. Most evident in the tests of
    # rosstage.
    time.sleep(0.75)
    try:
        rostest.run('rostest', 'base_long_time_test', HardwareTest, sys.argv)
    except KeyboardInterrupt, e:
        pass
    print "exiting"

