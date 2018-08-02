#!/usr/bin/python
import rospy
import sys

from cob_light.srv import SetLightMode, SetLightModeRequest, SetLightModeResponse

from actionlib import SimpleActionClient, GoalStatus

from std_msgs.msg import ColorRGBA

class TestLight():
    def __init__(self):
        self._srv_light_torso = rospy.ServiceProxy('/light_torso/set_light', SetLightMode)
        self._srv_light_base = rospy.ServiceProxy('/light_base/set_light', SetLightMode)
        self._light_mode_req1 = None
        self._light_mode_req2 = None
        self.setup_lightmode1()


    def setup_lightmode1(self):
        self._light_mode_req1 = SetLightModeRequest()
        self._light_mode_req1.mode.mode = 1
        self._light_mode_req1.mode.timeout = 3.0
        self._light_mode_req1.mode.priority = 12

    def execute(self, component):
        colors = [ColorRGBA(1.0,1.0,1.0,1.0), ColorRGBA(1.0,0.0,0.0,1.0), ColorRGBA(0.0,1.0,0.0,1.0), ColorRGBA(0.0,0.0,1.0,1.0)]
        for color in colors:
            rospy.loginfo("Setting lights to color (rgba):\n"+str(color))
            self._light_mode_req1.mode.colors.append(color)
            try:
                if (component == "torso"):
                    self._srv_light_torso.call(self._light_mode_req1)
                elif (component == "base"):
                    self._srv_light_base.call(self._light_mode_req1)
            except Exception as e:
                rospy.logerr("%s",e)
                pass
            self._light_mode_req1.mode.colors.remove(color)
            rospy.sleep(self._light_mode_req1.mode.timeout)
        return
        
class TestLightBase():
    def __init__(self):
        self._srv_light_base = rospy.ServiceProxy('/light_base/set_light', SetLightMode)
        self._light_mode_req1 = None
        self._light_mode_req2 = None
        self.setup_lightmode1()
        self.setup_lightmode2()

    def setup_lightmode1(self):
        self._light_mode_req1 = SetLightModeRequest()
        self._light_mode_req1.mode.mode=2
        self._light_mode_req1.mode.frequency = 3.0
        self._light_mode_req1.mode.priority = 12
        self._light_mode_req1.mode.pulses = 4
        self._light_mode_req1.mode.colors.append(ColorRGBA(1.0,1.0,1.0,1.0))
        self._light_mode_req1.mode.colors.append(ColorRGBA(1.0,0.0,0.0,1.0))
        self._light_mode_req1.mode.colors.append(ColorRGBA(0.0,1.0,0.0,1.0))
        self._light_mode_req1.mode.colors.append(ColorRGBA(0.0,0.0,1.0,1.0))

    def setup_lightmode2(self):
        self._light_mode_req2 = SetLightModeRequest()
        self._light_mode_req2.mode.mode=4
        self._light_mode_req2.mode.frequency = 0.5
        self._light_mode_req2.mode.priority = 12
        self._light_mode_req2.mode.timeout = 4.0
        self._light_mode_req2.mode.colors.append(ColorRGBA(1.0,1.0,1.0,1.0))

    def execute(self):
        print "light mode req:", self._light_mode_req1
        try:
            self._srv_light_base.call(self._light_mode_req1)
            rospy.sleep(3)
            self._srv_light_base.call(self._light_mode_req2)
            rospy.sleep(3)
        except Exception as e:
            smach.logerr("%s",e)
            pass

        return
        
if __name__ == '__main__':
    rospy.init_node('test_light')
    rospy.sleep(5)
    if (sys.argv[1] == "torso" or sys.argv[1] == "base"):
        test = TestLight()
        while not rospy.is_shutdown():
            test.execute(sys.argv[1])
    else:
        rospy.logerr("test_light: Wrong input argument(s)")
