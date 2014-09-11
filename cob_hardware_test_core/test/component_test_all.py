#!/usr/bin/env python
import sys
import time
#import math

# ROS imports
import roslib
#roslib.load_manifest('cob_hardware_test_core')
import rospy

# care-o-bot includes
#from simple_script_server import *
#from dialog_client import *


from helper import ComponentTest

## DONE:
	## TODO: Check that all the parameters (targets, topic, etc..) are set properly for each component
	## TODO: Check that at least one of the components (base, actuator) is received from param server
	## TODO: Save log for 'Target position out of error range'
	## TODO: Time limit for move_base function. Excecute sss.stop() if too much time passed
	## TODO: Add not rospy.is_shutdown() condition to every loop
	## TODO: Test summary (how many tests performed per component and how many fails etc..)
	## TODO: Stop the test if actuator fails (after trying to recover)

## UNDONE:
	## TODO: Change to seconds
	## TODO: Add summary of all components all fails and recovers
	## TODO: Add possibility of maximum rounds. At least one of these must be passed: max_time or max_rounds
	## TODO: Add init function to helper class


def run():
	test = ComponentTest('long_time_test')
	test.test_trigger_server()
	test.test_on = True
	test_count = 0
	error = False
	error_recover = False
	
	# Init components
	for component in test.actuators:
		if not test.init_component(component['name']):
			message = ('Failed to initialize component <<%s>>'
					   '\nerrorCode: %s'
					   %(component['name'], handle.get_error_code))
			test.log_diagnostics(component['name'], message)
			error = True
			break
	
	
	# Test loop
	duration = rospy.Time.now() + rospy.Duration(test.test_duration)
	while duration > rospy.Time.now() and not error and not test.toplevel_error:
		test.log_file.write('\n\n\n[ROUND %s] [%s]' %(test_count, rospy.Time.now()))
		
		tts = rospy.Time.now()	# Get the starting time of the loop for logging purpose
		
		# Move base
		if test.base_params:
			
			ts = rospy.Time.now()
			result, message = test.move_base(test.base_params)
			test.log_duration(component['name'], ts)
			
			if not result:
				message = ('Failed to move component <<base>>'
							   '\nerrorCode: %s'
							   %(component['name'], error_code))
				test.log_diagnostics(message)
				fail_diagnostics = test.get_diagnostics_agg()
				error = True
				break
			if test.toplevel_error: break
			
			
		# Move actuators	
		if test.actuators and not error and not test.toplevel_error:
			for component in test.actuators:
				
				ts = rospy.Time.now()
				result, message = test.move_actuator(component)
				test.log_duration(component['name'], ts)
				
				if not result:
					error = True
					if test.try_recover():
						test.log_diagnostics('Fail occurred while moving component <<%s>>. Recovered all components and trying to move the component again...' %(component['name']))
						ts = rospy.Time.now()
						result, message = test.move_actuator(component)
						test.log_duration(component['name'], ts)
						if result:
							component['recovered_tests'] += 1
							error_recover = True
							error = False
					if error == True:
						test.log_diagnostics(message)
						fail_diagnostics = test.get_diagnostics_agg()
						break
				if test.toplevel_error: break
				component['performed_tests'] += 1
				
		if not test.toplevel_error and not error:
			test.log_duration('Total', tts)
			
		test_count += 1
		
	
	
	
	test.test_on = False
	
	if test.toplevel_error and not error:
		message = ('Test has been terminated due to toplevel_state error!'
				   '\nToplevel_state: <<%s>>' %(test.toplevel_state))
		test.log_diagnostics(message)
	
	if test.toplevel_error or error:
		test.log_file.write(fail_diagnostics)
	
	
	
	
	### PRINT SUMMARY ###
	
	test.print_topic('SUMMARY')
	
	number_of_fails = 0
	number_of_components = 0
	for component in test.actuators:
		if component['recovered_tests'] > 0:
			number_of_fails += 1
		number_of_components += 1
	if test.base_params:
		if test.base_params['recovered_tests'] > 0:
			number_of_fails += 1
		number_of_components += 1
	
	
	
	if test.toplevel_error or error:
		test.log_file.write('\nTEST FAILED! \nTest has been terminated due to error!')
	elif number_of_fails > 0:
		test.log_file.write('\nTEST FAILED! \nNot all components passed the test without errors!')
	else:
		test.log_file.write('\nTEST SUCCEEDED! \nAll components passed the test without errors.')
	
	test.log_file.write('\n\nNumber of performed test rounds: %s' %(test_count))
	number_of_fails = 0
	number_of_components = 0
	for component in test.actuators:
		if component['recovered_tests'] > 0:
			number_of_fails += 1
		number_of_components += 1
	if test.base_params:
		if test.base_params['recovered_tests'] > 0:
			number_of_fails += 1
		number_of_components += 1
	test.log_file.write('\nIn total, %s out of %s components failed the test.' %(number_of_fails, number_of_components))
	
	
	test.log_file.write('\n\nNumber of performed tests: ')
	if test.base_params:
		test.log_file.write('\n  base: \t' + test.base_params['performed_tests'])
	for component in test.actuators:
		test.log_file.write('\n  %s: \t%s' %(component['name'], component['performed_tests']))
	
	test.log_file.write('\n\nNumber of failed but recovered and continued tests: ')
	if test.base_params:
		test.log_file.write('\n  base: \t' + test.base_params['recovered_tests'])
	for component in test.actuators:
		test.log_file.write('\n  %s: \t%s' %(component['name'], component['recovered_tests']))
	
	
	
	
	rospy.sleep(1)
	test.log_file.close()
		
if __name__ == "__main__":
	try:
		run()
	except KeyboardInterrupt, e:
		pass
	print "exiting"
