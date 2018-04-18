#!/usr/bin/env python
#
# Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import sys
import math
import random
from optparse import OptionParser

import rospy
from std_msgs.msg import Float64

def pubSine(options):
    rospy.init_node("test_single_sine", anonymous=True)

    pub = rospy.Publisher("command", Float64, queue_size=1)
    rospy.sleep(1.0)

    freq = 100.0
    r = rospy.Rate(freq)

    a = float(options.a)
    b = float(options.b) * (2.0*math.pi/freq)
    c = float(options.c)
    d = float(options.d)
    n = float(options.n)
    i = 0.0

    rospy.loginfo("Generating commands for: %s*sine(2pi*%sx+%s)+%s", str(options.a), str(options.b), str(options.c), str(options.d))
    rospy.loginfo("Adding noise: %s", str(options.n))
    input_msg = Float64()

    while not rospy.is_shutdown():
        input_msg.data = a*math.sin(b*i+c) + d

        noise = random.uniform(-n, n)
        input_msg.data += noise

        pub.publish(input_msg)

        i += 1.0
        r.sleep()

if __name__ == '__main__':
    _usage = """%prog [options]
    type %prog -h for more info."""

    parser = OptionParser(usage=_usage, prog=os.path.basename(sys.argv[0]))
    parser.add_option(
        '-a', '--amplitude', dest='a', default=1.0,
        help="a in general form of sine function a*sin(2pi*bx+c)+d, default 1.0")
    parser.add_option(
        '-b', '--frequency', dest='b', default=0.1,
        help="b in general form of sine function a*sin(2pi*bx+c)+d, default: 0.1")
    parser.add_option(
        '-c', '--phase', dest='c', default=0.0,
        help="c in general form of sine function a*sin(2pi*bx+c)+d, default: 0.0")
    parser.add_option(
        '-d', '--offset', dest='d', default=0.0,
        help="d in general form of sine function a*sin(2pi*bx+c)+d, default: 0.0")
    parser.add_option(
        '-n', '--noise', dest='n', default=0.0,
        help="additional white noise, default: 0.0")
    (options, args) = parser.parse_args()
    try:
        pubSine(options)
    except rospy.ROSInterruptException: pass
