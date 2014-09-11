#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test_core')
import sys
import time
import rospy
import math
import collections

# msg & srv imports
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from pr2_controllers_msgs.msg import *
from diagnostic_msgs.msg import *
from cob_relayboard.msg import *
from cob_hardware_test_core.srv import TestTrigger

# care-o-bot includes
from simple_script_server import *
from dialog_client import *



class ComponentTest:
	def __init__(self, test_type):
		
		self.sss = simple_script_server()

		### GET PARAMETERS ###
		self.base_params = None
		self.actuators = []
		self.sensors = []

		
		if test_type == 'long_time_test':
			self.init_long_time_test()
		elif test_type == 'daily_morning_show':
			self.init_daily_morning_show()
	
	
	def init_daily_morning_show(self):
		
		rospy.init_node('daily_morning_show')
		
		### INTERNAL PARAMETERS
		self.wait_time = 1		# waiting time (in seconds) before trying initialization again
		self.wait_time_recover = 1
		self.wait_time_diag = 3
		###

		### GET PARAMETERS ###
		# Get base goals
		try:
			params_base = rospy.get_param('/morning_show/components/base/goals')
			self.base_params = params_base
		except:
			pass
			
		# Get actuator parameters
		try:
			params_actuator = rospy.get_param('/morning_show/components/actuators')
			for k in params_actuator.keys():
				self.actuators.append(params_actuator[k])
				
		except:
			raise NameError('##########')
		
		# Get sensor parameters	
		try:
			params = rospy.get_param('/morning_show/components/sensors')
			for k in params.keys():
				self.sensors.append(params[k])
		except:
			pass
			
		if not self.base_params and not self.actuators and not self.sensors:
			raise NameError('Couldn\'t find any components to test under /component_test namespace. '
							'You need to define at least one component in order to run the program (base, actuator, sensor). '
							'View the documentation to see how to define test-components.')
		
		
		# Get log-file directory
		try:
			log_dir = rospy.get_param('/morning_show/result_dir')
		except:
			raise NameError('Test-result directory not set.')
			
		# Create and prepare logfile
		complete_name = '%s/daily_show_results_%s.txt' %(log_dir, time.strftime("%Y%m%d"))
		self.log_file = open(complete_name,'w')
		
		
		self.log_file.write('Daily-show test %s' %(time.strftime('%d.%m.%Y')))
		self.log_file.write('\n\nTested components:')
		if self.base_params:
			self.log_file.write('\n  base')
		if self.actuators:
			for actuator in self.actuators:
				self.log_file.write('\n  ' + actuator['name'])
		
		
		
		self.print_topic('TEST PARAMETERS')
		params = rospy.get_param('/morning_show/components')
		for key in params:
			self.log_file.write('\n%s:' %(key))
			params_2 = rospy.get_param('/morning_show/components/%s' %(key))
			for key_2 in params_2:
				self.log_file.write('\n  %s:' %(key_2))
				params_3 = rospy.get_param('/morning_show/components/%s/%s' %(key, key_2))
				for key_3, value in params_3.iteritems():
					self.log_file.write('\n    %s: %s' %(key_3, value))
					
		self.print_topic('TEST LOG')
	
	
	def init_long_time_test(self):
		rospy.init_node('component_test_all')
		
		### GET PARAMETERS ###
		self.wait_time = 3			# waiting time (in seconds) for state message
		self.wait_time_recover = 1	
		
		# Message types
		self.actuator_msg = JointTrajectoryControllerState
		self.scan_msg = LaserScan
		self.kinect_msg = PointCloud2
		self.image_msg = Image
		
		self.msg_received = False
		self.sss = simple_script_server()	
		
		# Get base goals
		try:
			params_base = rospy.get_param('/component_test/components/base/goals')
			self.base_params = params_base
			self.base_params['performed_tests'] = 0
			self.base_params['recovered_tests'] = 0
		except:
			pass
			
		# Get actuator parameters
		try:
			params_actuator = rospy.get_param('/component_test/components/actuators')
			for k in params_actuator.keys():
				params_actuator[k]['performed_tests'] = 0
				params_actuator[k]['recovered_tests'] = 0
				self.actuators.append(params_actuator[k])
		except:
			pass
			
		# Get sensor parameters	
		try:
			params = rospy.get_param('/component_test/components/sensors')
			for k in params.keys():
				self.sensors.append(params[k])
		except:
			pass
			
		if not self.base_params and not self.actuators and not self.sensors:
			raise NameError('Couldn\'t find any components to test under /component_test namespace. '
							'You need to define at least one component in order to run the program (base, actuator, sensor). '
							'View the documentation to see how to define test-components.')
		
			
		# Get test duration
		try:
			self.test_duration = 60.0 * float(rospy.get_param('/component_test/test_duration'))
		except:
			raise NameError('Test duration not set or set improperly. Please give test duration in minutes as an integer.')
		
		# Get log-file directory
		try:
			log_dir = rospy.get_param('/component_test/result_dir')
		except:
			raise NameError('Test-result directory not set.')
		
		
		# Subscribe to toplevel-state topic
		self.toplevel_state_received = False
		sub_top_state = rospy.Subscriber("/diagnostics_toplevel_state", DiagnosticStatus, self.cb_toplevel_state)
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		while not self.toplevel_state_received and rospy.get_rostime() < abort_time:
			rospy.sleep(0.1)
		if not self.toplevel_state_received:
			raise NameError('Could not subscribe to "/diagnostics_toplevel_state" topic')
		self.toplevel_error = False
		
		
		# Create and prepare log file		
		if self.base_params or self.actuators:
			complete_name = '%s/component_test_results_%s.txt' %(log_dir, time.strftime("%Y%m%d"))
			self.log_file = open(complete_name,'w')
			
			# Log all tested components
			self.log_file.write('Component test %s' %(time.strftime('%d.%m.%Y')))
			self.print_topic('TESTED COMPONENTS')
			if self.base_params:
				self.log_file.write('\n  base\n')
			if self.actuators:
				for actuator in self.actuators:
					self.log_file.write('\n  ' + actuator['name'])
					
			# Log test parameters
			self.print_topic('TEST PARAMETERS')
			params = rospy.get_param('/component_test/components')
			for key in params:
				self.log_file.write('\n%s:' %(key))
				params_2 = rospy.get_param('/component_test/components/%s' %(key))
				for key_2 in params_2:
					self.log_file.write('\n  %s:' %(key_2))
					params_3 = rospy.get_param('/component_test/components/%s/%s' %(key, key_2))
					for key_3, value in params_3.iteritems():
						self.log_file.write('\n    %s: %s' %(key_3, value))
			
			
			self.print_topic('TEST LOG')
			
			#log starting time
			self.log_file.write('\n[INFO] [%s] [%s]'
								'\n  Test started.' 
								%(time.strftime('%H:%M:%S'), rospy.Time.now()))
	
	
	def init_component(self, component):
		handle = self.sss.init(component)
		if handle.get_error_code() == 0:
			return True
		return False
	
	
	def move_actuator(self, component):
		
		# Move to test position
		move_handle = self.sss.move(component['name'], component['test_target'])
		if move_handle.get_state() != 3:
			message = ('Failed to move component <<%s>> to target position'
					   '\nerrorCode: %s'
					   %(component['name'], move_handle.get_error_code()))
			return (False, message)
		move_handle.wait()
		if move_handle.get_error_code() != 0:
			message = ('Error occurred while moving component <<%s>> to target position'
					   '\nerrorCode: %s'
					   %(component['name'], move_handle.get_error_code()))
			return (False, message)
		
		# Move to default position
		move_handle = self.sss.move(component['name'], component['default_target'])
		if move_handle.get_state() != 3:
			message = ('Failed to move component <<%s>> to default position'
					   '\nerrorCode: %s'
					   %(component['name'], move_handle.get_error_code()))
			return (False, message)
		move_handle.wait()
		if move_handle.get_error_code() != 0:
			message = ('Error occurred while moving component <<%s>> to default position'
					   '\nerrorCode: %s'
					   %(component['name'], move_handle.get_error_code()))
			return (False, message)
		
		return (True, 0)
	
	
	def recover_test(self):
		dialog_client(0, 'Please activate and release the EM Stop. Press "OK" AFTER releasing the EM Stop.')
		while self.check_em_stop():
			dialog_client(0, 'Release the EM-stop and press ''OK''')
		
		recover_handle = []
		if self.base_params:
			recover_handle.append(self.sss.recover('base'))
		for component in self.actuators:
			recover_handle.append(self.sss.recover(component['name']))
		rospy.sleep(self.wait_time_recover)
		
		i=0
		for component in self.actuators:
			self.log_file.write('\n  %s: ' %(component['name']))
			if recover_handle[i].get_error_code() == 0:
				self.log_file.write('<<OK>>')
			else:
				self.log_file.write('<<FAIL>>')
			i+=1		
	
	
	def move_base(self, base_params):
		i = 0
		while True:
			next_goal = 'test_%s'%(i)
			if next_goal in base_params:
				move_handle = self.sss.move("base", base_params[next_goal])
				if move_handle.get_state() != 3:
					message = ('Could not move base to position <<%s>>'
							   '\nerrorCode: %s'
							   %(next_goal, move_handle.get_error_code()))
					return (False, message)
					
				move_handle.wait()
				
				if move_handle.get_error_code() != 0:
					message = ('Could not move base to position <<%s>>'
							   '\nerrorCode: %s'
							   %(next_goal, move_handle.get_error_code()))
					return (False, message)
				i += 1
			else: break
			
		# Back to default position
		move_handle = self.sss.move("base",base_params['test_0'])
		if move_handle.get_state() != 3:
			message = ('Could not move base to default position <<test_0>>'
							   '\nerrorCode: %s'
							   %(move_handle.get_error_code()))
			return (False, message)
			
		move_handle.wait()	
		
		if move_handle.get_error_code() != 0:
			message = ('Could not move base to default position <<test_0>>'
					   '\nerrorCode: %s'
					   %(next_goal, move_handle.get_error_code()))
			return (False, message)	
		return (True, None)
	
	
	def move_base_rel(self, goals):
		i = 0
		while True:
			next_goal = 'test_%s'%(i)
			move_handle = self.sss.move_base_rel('base', next_goal)
			
			if next_goal in goals:
				if move_handle.get_error_code() != 0 or not self.dialog('base', next_goal):
					return False
				i += 1
			else: return True
	
	
	def check_em_stop(self):
		self.em_stop_pressed = False
		self.em_msg_received = False
		sub_em_stop = rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.cb_em_stop)
		
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		while not self.em_msg_received and rospy.get_rostime() < abort_time:
			rospy.sleep(0.1)
		rospy.sleep(1.0)
		sub_em_stop.unregister()
		if self.em_stop_pressed:
			return True
		return False
	
	
	def check_sensor(self, topic, msg_type):
		self.msg_received = False
		
		if str(msg_type) == "JointTrajectoryControllerState": cb_func = self.cb_actuator
		elif msg_type == "LaserScan": cb_func = self.cb_scanner
		elif msg_type == "PointCloud2": cb_func = self.cb_point_cloud
		elif msg_type == "Image": cb_func = self.cb_camera
		else: raise NameError('Unknown message! No callback function defined for message type <<%s>>' %(msg_type))
		
		sub_topic = rospy.Subscriber(topic, eval(msg_type), cb_func)
		
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		while not self.msg_received and rospy.get_rostime() < abort_time:
			rospy.sleep(0.1)
			
		sub_topic.unregister()
		
		if self.msg_received:
			return True
		return False
	
	
	
	def dialog(self, component, goal):
		if dialog_client(1, 'Did component <<%s>> move to position <<%s>>?' %(component, goal)):
			return True
		return False
	
	
	
	def print_topic(self, text):
		topic = '\n\n\n'
		for i in enumerate(text):
			topic += '='
		topic += '\n%s\n' %(text)
		for i in enumerate(text):
			topic += '='
		topic += '\n'
		self.log_file.write(topic)
	

	
	def get_diagnostics(self, component):
		
		self.log_file.write('\n    Diagnostics:')
		
		self.msg_diag_received = False
		sub_diagnostics = rospy.Subscriber("/diagnostics", DiagnosticArray, self.cb_diagnostics)
		# Wait for the message
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time_diag)
		while not self.msg_diag_received and rospy.get_rostime() < abort_time:
			rospy.sleep(0.1)
		
		if self.msg_diag_received:
			# Get diagnostics from /diagnostics topic
			name = '%s_controller' %(component)
			diag_name = ""
			abort_time = rospy.Time.now() + rospy.Duration(self.wait_time_diag)
			while diag_name != name and rospy.get_rostime() < abort_time:
				diagnostics = str(self.diagnostics_status)
				diag_name = diagnostics.replace('/','')
				diag_name = (diag_name.split('name: ', 1)[1]).split('\n',1)[0]
			sub_diagnostics.unregister()
			
			if diag_name == name:
				diagnostics = diagnostics.replace('[','')
				diagnostics = diagnostics.replace(']','')
				diagnostics = '      ' + diagnostics.replace('\n','\n      ')
				self.log_file.write('\n' + diagnostics)
			else:
				self.log_file.write('\n      No diagnostics found by name <<%s>>' %(name))
		else:
			self.log_file.write('\n      Could not subscribe to /diagnostics topic')
	
	
	
	def try_recover(self):
		recover_handle = []
		if self.base_params:
			recover_handle.append(self.sss.recover('base'))
		for component in self.actuators:
			recover_handle.append(self.sss.recover(component['name']))
		rospy.sleep(self.wait_time_recover)
		i=0
		for component in self.actuators:
			if recover_handle[i].get_error_code() != 0:
				return False
		return True



	def log_diagnostics(self, message):
			
		rtstamp = rospy.Time.now()
		self.log_file.write('\n\n  [FAIL] \t[%s]\n' %(rtstamp))
		message = '    ' + message.replace("\n","\n  ")
		self.log_file.write(message + '\n')
	
	
	
	def get_diagnostics_agg(self):
		
		self.print_topic('DIAGNOSTICS')
		### GET DIAGNOSTICS ###	
		# Wait for the message
		self.msg_diag_received = False
		sub_diagnostics = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.cb_diagnostics)
		
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		while not self.msg_diag_received and rospy.get_rostime() < abort_time:
			rospy.sleep(0.1)
		
		if self.msg_diag_received:
			# Get diagnostics from /diagnostics_agg topic
			diagnostics = str(self.diagnostics_status)
			sub_diagnostics.unregister()
			return ('\n\n%s' %(diagnostics))
		else:
			return '\n\nCould not subscribe to /diagnostics_agg topic'
	
	
	
	def log_duration(self, component, start_time):
		duration = rospy.Time.now() - start_time
		self.log_file.write('\n  %s: \t[%s]' %(component, duration))
		
		
		
	def handle_test_trigger(self, req):
		return self.test_on

	def test_trigger_server(self):
		s = rospy.Service('test_trigger', TestTrigger, self.handle_test_trigger)
		
	
	
	
	
	
	
	
	### CALLBACKS ###
	def cb_em_stop(self, msg):
		self.em_stop_pressed = msg.emergency_button_stop
		self.em_msg_received = True
		
	def cb_actuator(self, msg):
		self.actuator_position = msg.actual.positions
		self.msg_received = True
		
	def cb_diagnostics(self, msg):
		self.diagnostics_status = msg.status
		self.msg_diag_received = True
		
	def cb_actuator(self, msg):
		self.actuator_msg = msg.actual.positions
		self.msg_received = True

	def cb_scanner(self, msg):
		self.scanner_msg = msg.ranges
		self.msg_received = True

	def cb_point_cloud(self, msg):
		self.point_cloud_msg = msg.fields
		self.msg_received = True

	def cb_camera(self, msg):
		self.camera_msg = msg.data
		self.msg_received = True
		
	def cb_toplevel_state(self, msg):
		self.toplevel_state = msg.level
		if self.toplevel_state != 2:
			self.toplevel_error = True
		self.toplevel_state_received = True
		
