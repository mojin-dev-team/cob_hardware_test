#!/usr/bin/env python
import sys
import time

# ROS imports
import roslib
import rospy


from helper import ComponentTest
from dialog_client import *


def run():
	test = ComponentTest('long_time_test')
	test.test_trigger_server()
	test.test_on = True
	test_count = 0
	error = False
	error_recover = False
	fail_diagnostics = " "
	message= " "

    # Check if the robot is running as a simulation
	#is_sim = dialog_client(1, 'Is the robot running as a simulation?')
	is_sim = False
	# Test base?!
	test_base = True
	
	if not is_sim :	
		# Init base
		if test.base_params:
			if not test.init_component('base'):
				test.log_diagnostics('Failed to initialize component <<base>>')
				error = True
	
		# Init components
		for component in test.actuators:
			if not test.init_component(component['name']):
				test.log_diagnostics('Failed to initialize component <<%s>>')
				error = True
				break
	
	
	# Test loop
	duration = rospy.Time.now() + rospy.Duration(test.test_duration)
	while rospy.Time.now() < duration and test_count < test.test_rounds and not error and not test.toplevel_error:
		test.log_file.write('\n\n\n[ROUND %s] [%s]' %(test_count, time.strftime('%H:%M:%S')))
		tts = rospy.Time.now()	# Get the starting time of the loop for logging purpose
		
		
		# Move base
		error_recover = False		
		if test.base_params and not is_sim and test_base:
			i = 0
			ts = rospy.Time.now()
			
			while True:
				next_goal = 'test_%s'%(i)
				max_dur = test.base_params['max_duration']
				
				if next_goal in test.base_params:
					result, message = test.move_base(next_goal, max_dur)
					if not result:
						test.log_duration('base', ts)
						test.log_diagnostics(message)
						if not error_recover:
							test.log_diagnostics('Trying to recover all components and move the component again...')
							if test.try_recover():
								test.log_diagnostics('Recovered all components.')
								ts = rospy.Time.now()
								result, message = test.move_base(next_goal, max_dur)
								if result:
									test.base_params['recovered_tests'] += 1
									error_recover = True
								else:
									test.log_duration('base', ts)
									test.log_diagnostics(message)
									fail_diagnostics = test.get_diagnostics_agg()
									error = True
									break
							else:
								test.log_diagnostics('Could not recover components.')
								error = True
								break
						else:
							error = True
							break
							
					if test.toplevel_error: break
				else:
					test.log_duration('base', ts)
					break
				i += 1
					
			test.base_params['performed_tests'] += 1
			if error: test.base_params['failed_tests'] += 1
			
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
							error = False
					if error == True:
						test.log_diagnostics(message)
						fail_diagnostics = test.get_diagnostics_agg()
						component['failed_tests'] += 1
						break
				if test.toplevel_error: break
				component['performed_tests'] += 1
				
		if not test.toplevel_error and not error:
			test.log_duration('Total', tts)
			test_count += 1
			
	if error:
		test.log_diagnostics('Terminating test...')
	
	test.test_on = False
	
	if test.toplevel_error and not error:
		message = ('Test has been terminated due to toplevel_state error!'
				   '\nToplevel_state: <<%s>>' %(test.toplevel_state))
		test.log_diagnostics(message)
	
	if test.toplevel_error or error:
		test.print_topic('DIAGNOSTICS')
		test.log_file.write(fail_diagnostics)
	
	
	
	
	### PRINT SUMMARY ###
	
	if is_sim:
		test.print_topic('SUMMARY(Simulation)')
	else:
		test.print_topic('SUMMARY')
	
	number_of_fails = 0
	number_of_components = 0
	for component in test.actuators:
		if component['recovered_tests'] > 0 or component['failed_tests'] > 0:
			number_of_fails += 1
		number_of_components += 1
	if test.base_params:
		if test.base_params['recovered_tests'] > 0 or test.base_params['failed_tests'] > 0:
			number_of_fails += 1
		number_of_components += 1
	
	
	
	if test.toplevel_error or error:
		test.log_file.write('\nTEST FAILED! \n\nTest has been terminated due to following error: \n' + message)
	elif number_of_fails > 0:
		test.log_file.write('\nTEST FAILED! \n\nNot every component passed the test without errors!')
	else:
		test.log_file.write('\nTEST WAS SUCCESSFUL! \n\nAll components passed the test without errors.')
	
	test.log_file.write('\n\nNumber of performed test rounds: %s' %(test_count))
	number_of_fails = 0
	number_of_components = 0
	for component in test.actuators:
		if component['recovered_tests'] > 0 or component['failed_tests'] > 0:
			number_of_fails += 1
		number_of_components += 1
	if test.base_params:
		if test.base_params['recovered_tests'] > 0 or test.base_params['failed_tests'] > 0:
			number_of_fails += 1
		number_of_components += 1
	if number_of_fails > 0:
		test.log_file.write('\n%s out of %s components failed the test.' %(number_of_fails, number_of_components))
		
	
	test.log_file.write('\n\nNumber of performed tests: ')
	if test.base_params:
		test.log_file.write('\n  base: \t%s' %(test.base_params['performed_tests']))
	for component in test.actuators:
		test.log_file.write('\n  %s: \t%s' %(component['name'], component['performed_tests']))
	
	test.log_file.write('\n\nNumber of failed but recovered and continued tests: ')
	if test.base_params:
		test.log_file.write('\n  base: \t%s' %(test.base_params['recovered_tests']))
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
