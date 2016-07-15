#!/usr/bin/env python
import roslib
import sys
import rospy
import time

from helper import ComponentTest
from simple_script_server import *
from dialog_client import *
#from light_test import HardwareTest
	

def run():
	test = ComponentTest('daily_morning_show')
	#light_test = HardwareTest('daily_morning_show_lights')
	sss = simple_script_server()
	
	# Check that em_stop is not pressed
	while test.check_em_stop():
		dialog_client(0, 'Release EM-stop and press ''OK''')
		
	# Initialize
	test.log_file.write('\n[INITIALIZE] [%s]' %(time.strftime('%H:%M:%S')))
	if test.base_params:
		test.log_file.write('\n  base: ')
		if test.init_component('base'):
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')
	
	for component in test.actuators:
		test.log_file.write('\n  %s: ' %(component['name']))
		if test.init_component(component['name']):
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')
	
	rospy.sleep(1)
	
	#### MOVE TEST 1 ###
	test.log_file.write('\n\n[MOVE_TEST_1] [%s]' %(time.strftime('%H:%M:%S')))
	if test.base_params:
		# Move base
		test.log_file.write('\n  base: ')
		if test.move_base_rel(test.base_params) and dialog_client(1,'Did the robot move?'):
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')
			test.get_diagnostics('base')
	
	# Move actuators
	for component in test.actuators:
		test.log_file.write('\n  %s: ' %(component['name']))
		if test.move_actuator_daily(component) and test.dialog(component['name'], component['test_target']):
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')
			test.get_diagnostics(component['name'])						
	
	### RECOVER TEST ###
	test.log_file.write('\n\n[RECOVER_TEST] [%s]' %(time.strftime('%H:%M:%S')))
	test.recover_test()
	
	
	### MOVE TEST 2 ###
	test.log_file.write('\n\n[MOVE_TEST_2] [%s]' %(time.strftime('%H:%M:%S')))
	if test.base_params:
		# Move base
		test.log_file.write('\n  base: ')
		if test.move_base_rel(test.base_params) and dialog_client(1,'Did the robot move?'):
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')
			test.get_diagnostics('base')
	
	# Move actuators
	for component in test.actuators:
		test.log_file.write('\n  %s: ' %(component['name']))
		if test.move_actuator_daily(component) and test.dialog(component['name'], component['test_target']):
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')
			test.get_diagnostics(component['name'])
	
	
	
	# Change lights, test mimics and sound
	# This part is completely "hardcoded", only use with Cob4!
	if True:
		test.log_file.write('\n\n[MIMICS, SOUNDS & LEDS] [%s]' %(time.strftime('%H:%M:%S')))
		test.log_file.write('\n  mimics:')
		sss.set_mimic("mimic","asking")
		rospy.sleep(1)
		mimic_working = dialog_client(1,'Do you see the question-mark?')
		sss.set_mimic("mimic","default")
		if mimic_working:
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')
		test.log_file.write('\n  sound:')
		sss.say(["test 1, 2, 3"])
		rospy.sleep(1)
		sound_working = dialog_client(1,'Did you hear me speak? (Next: Lights, so pay attention!)')
		if sound_working:
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')
		test.log_file.write('\n  LEDs:')
		sss.set_light("light_base","cyan")
		sss.set_light("light_torso","cyan")
		rospy.sleep(1.0)
		sss.set_light("light_base","yellow")
		sss.set_light("light_torso","yellow")
		rospy.sleep(3.0)
		sss.set_light("light_base","red")
		sss.set_light("light_torso","red")
		rospy.sleep(3.0)
		sss.set_light("light_base","cyan")
		sss.set_light("light_torso","cyan")
		rospy.sleep(3.0)
		lights_changed = dialog_client(1, 'Did you see the lights change from "cyan" to "yellow" to "red" and back?')
		if lights_changed:
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')
	
	# Sensor test
	test.log_file.write('\n\n[SENSOR_TEST] [%s]' %(time.strftime('%H:%M:%S')))
	
	for sensor in test.sensors:
		test.log_file.write('\n  %s: ' %(sensor['name']))
		
		if test.check_sensor(sensor['topic'], sensor['msg_type']):
			test.log_file.write('<<OK>>')
		else:
			test.log_file.write('<<NO_MSG>>')	
	
	
	test.log_file.close()


if __name__ == '__main__':
	
	try:
		run()
	except KeyboardInterrupt, e:
		pass
	print "exiting"
