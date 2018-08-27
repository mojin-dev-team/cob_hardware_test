#!/usr/bin/python
import rospy
import sys

from cob_light.srv import SetLightMode, SetLightModeRequest, SetLightModeResponse
from cob_light.msg import LightModes

from actionlib import SimpleActionClient, GoalStatus

from std_msgs.msg import ColorRGBA

class TestLight():
    def __init__(self):
        self._srv = rospy.ServiceProxy('set_light', SetLightMode)
        self._num_leds = rospy.get_param("num_leds", 1)

    def execute(self):
        req = SetLightModeRequest()
        req.mode.mode = LightModes.STATIC
        req.mode.timeout = 3.0
        req.mode.priority = 12
        req.mode.colors = [ColorRGBA(1.0,1.0,1.0,1.0)] * self._num_leds
        colors = [ColorRGBA(1.0,0.0,0.0,1.0), ColorRGBA(0.0,1.0,0.0,1.0), ColorRGBA(0.0,0.0,1.0,1.0)]
        for color in colors:
            for i in range(self._num_leds):
            rospy.loginfo("Setting led " + str(i) + " to color (rgba):\n"+str(color))
            req.mode.colors[i] = color
            try:
                self._srv.call(req)
            except Exception as e:
                rospy.logerr("%s",e)
                pass
            rospy.sleep(req.mode.timeout)
        return
        
if __name__ == '__main__':
    rospy.init_node('test_light')
    rospy.sleep(5)
    test = TestLight()
    test.execute_single()
    while not rospy.is_shutdown():
        test.execute()
