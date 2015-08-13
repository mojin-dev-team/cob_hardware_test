#!/usr/bin/env python
import roslib
import sys
import rospy
import time

from helper import ComponentTest
#from light_test import HardwareTest
	

def run():
	test = ComponentTest('daily_morning_show')
	#light_test = HardwareTest('daily_morning_show_lights')
	
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
		if test.move_base_rel(test.base_params):
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
					
	# Change light #TODO
	if False:
		test.log_file.write('\n LEDs:')
		if light_test.test_set_light():
			test.log_file.write('\t<<OK>>')
		else:
			test.log_file.write('\t<<FAIL>>')

	
	### RECOVER TEST ###
	test.log_file.write('\n\n[RECOVER_TEST] [%s]' %(time.strftime('%H:%M:%S')))
	test.recover_test()
	
	
	### MOVE TEST 2 ###
	test.log_file.write('\n\n[MOVE_TEST_2] [%s]' %(time.strftime('%H:%M:%S')))
	if test.base_params:
		# Move base
		test.log_file.write('\n  base: ')
		if test.move_base_rel(test.base_params):
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
	
	
	
	# Sensor test
	test.log_file.write('\n\n[SENSOR_TEST] [%s]' %(time.strftime('%H:%M:%S')))
	
	for sensor in test.sensors:
		test.log_file.write('\n  %s: ' %(sensor['name']))
		#rospy.sleep(0.5)
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
