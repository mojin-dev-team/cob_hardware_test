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
        self._colors = [("red",ColorRGBA(1.0,0.0,0.0,1.0)), ("green",ColorRGBA(0.0,1.0,0.0,1.0)), ("blue", ColorRGBA(0.0,0.0,1.0,1.0))]
        self._timer = None
        self._idx = 0

    def start(self):
        self._timer = rospy.Timer(rospy.Duration(10), self.execute)

    def execute(self, event):
        rospy.loginfo("Setting color to %s",self._colors[self._idx%3][0])
        req = SetLightModeRequest()
        req.mode.mode = LightModes.CIRCLE_COLORS
        req.mode.timeout = 0.0
        req.mode.priority = 12
        req.mode.frequency = 1.0
        req.mode.colors = [self._colors[self._idx%3][1]]
        try:
            self._srv.call(req)
        except Exception as e:
            rospy.logerr("%s",e)
            pass
        self._idx += 1
        return

    def stop(self):
        self._timer.shutdown()
        rospy.loginfo("stopping...")
        req = SetLightModeRequest()
        req.mode.mode = LightModes.NONE
        req.mode.colors = [ColorRGBA()]
        try:
            self._srv.call(req)
        except Exception as e:
            pass
        return
        
if __name__ == '__main__':
    rospy.init_node('test_light')
    rospy.sleep(2)
    test = TestLight()
    test.start()
    rospy.spin()
    test.stop()
