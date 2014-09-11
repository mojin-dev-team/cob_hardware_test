#!/usr/bin/env python
import roslib
import sys
import rospy

from helper import ComponentTest



# DONE:
	# TODO: Implement error_code check for init and recover tests
	# TODO: delete manual launch component
	# TODO: Always use the diagnostics for errors and success cases
	# TODO: Use the aggrigated diagnostics
	# TODO: group all sensor tests
	# TODO: add base test  (base_rel)
	# TODO: add a dialog to ask the user if move was successful, also for the base
	
	
# UNDONE:
	# TODO: Combine check_sensor, check_msg etc...
	# TODO: Move Init and other functions to helper class, except run
	

def run():
	test = ComponentTest('daily_morning_show')
	
	# Check that em_stop is not pressed
	while test.check_em_stop():
		dialog_client(0, 'Release EM-stop and press ''OK''')
		
	# Initialize
	test.log_file.write('\n[Initialize] [%s]' %(rospy.Time.now()))
	if test.base_goals:
		test.log_file.write('\n  base: ')
		if test.init_component('base'):
			test.log_file.write('<<OK>>')
		else:
			test.log_file.write('<<FAIL>>')
	
	for component in test.actuators:
		test.log_file.write('\n  %s: ' %(component['name']))
		if test.init_component(component['name']):
			test.log_file.write('<<OK>>')
		else:
			test.log_file.write('<<FAIL>>')
	
	#### MOVE TEST 1 ###
	test.log_file.write('\n\n[Move_test_1] [%s]' %(rospy.Time.now()))
	# Move base
	test.log_file.write('\n  base: ')
	if test.move_base_rel(test.base_goals):
		test.log_file.write('<<OK>>')
	else:
		test.log_file.write('<<FAIL>>')
		test.get_diagnostics('base')
	
	# Move actuators
	for component in test.actuators:
		test.log_file.write('\n  %s: ' %(component['name']))
		if test.move_actuator(component) and test.dialog(component['name'], component['test_target']):
			test.log_file.write('<<OK>>')
		else:
			test.log_file.write('<<FAIL>>')
			test.get_diagnostics(component['name'])
	
	
	### RECOVER TEST ###
	test.log_file.write('\n\n[Recover_test] [%s]' %(rospy.Time.now()))
	test.recover_test()
	
	
	### MOVE TEST 2 ###
	test.log_file.write('\n\n[Move_test_2] [%s]' %(rospy.Time.now()))
	# Move base
	test.log_file.write('\n  base: ')
	if test.move_base_rel(test.base_goals):
		test.log_file.write('<<OK>>')
	else:
		test.log_file.write('<<FAIL>>')
		test.get_diagnostics('base')
	
	# Move actuators
	for component in test.actuators:
		test.log_file.write('\n  %s: ' %(component['name']))
		if test.move_actuator(component) and test.dialog(component['name'], component['test_target']):
			test.log_file.write('<<OK>>')
		else:
			test.log_file.write('<<FAIL>>')
			test.get_diagnostics(component['name'])
	
	
	
	# Sensor test
	test.log_file.write('\n\n[Sensor_test] [%s]' %(rospy.Time.now()))
	
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
