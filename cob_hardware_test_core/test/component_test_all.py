#!/usr/bin/env python
import sys
import time
import math

# ROS imports
import roslib
roslib.load_manifest('cob_hardware_test_core')
import rospy

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


class ComponentTestAll:
	def __init__(self):
		rospy.init_node('component_test_all')
		
		### PARAMETERS ###
		self.max_init_tries = 1		# maximum initialization tries for each component
		self.wait_time = 3			# waiting time (in seconds) before trying initialization again
		self.wait_time_diag = 1		# waiting time in seconds 
		self.test_numbers = 2		# number of test repeats
		
		# Message types
		self.actuator_msg = JointTrajectoryControllerState
		self.scan_msg = LaserScan
		self.kinect_msg = PointCloud2
		self.image_msg = Image
		
		self.msg_received = False
		self.sss = simple_script_server()		
		
		### GET PARAMETERS ###
		self.base_goals = None
		self.actuators = []
		self.sensors = []
		# Get base goals
		try:
			params_base = rospy.get_param('/component_test/base/goals')
			self.base_goals = params_base
		except:
			raise NameError('###############')
			
		# Get actuator parameters
		try:
			params_actuator = rospy.get_param('/component_test/actuators')
			i=0
			for k in params_actuator.keys():
				self.actuators.append(params_actuator[k])
				i+=1
		except:
			raise NameError('###############')
			
		# Get test duration
		try:
			self.test_duration = 60.0 * int(rospy.get_param('/component_test/test_duration'))
		except:
			raise NameError('Test duration not set or set improperly. Please give test duration in minutes as an integer.')
			
		
		# Get log-file directory
		try:
			log_dir = rospy.get_param('/component_test/result_dir')
		except:
			raise NameError('Test-result directory not set.')
			
		# Create logfile
		#complete_name = '/home/nhg-tl/Documents/AllComponentsTest/results/component_test_results_%s.txt' %(time.strftime("%Y%m%d"))
		complete_name = '%s/component_test_results_%s.txt' %(log_dir, time.strftime("%Y%m%d"))
		self.log_file = open(complete_name,'w')	
			
			
			
		# Subscribe to em-stop topic
		self.em_msg_received = False
		sub_em_stop = rospy.Subscriber("/emergency_stop_state", EmergencyStopState, self.cb_em_stop)
		em_stop_abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		while not self.em_msg_received and rospy.get_rostime() < em_stop_abort_time:
			rospy.sleep(0.1)
		if not self.em_msg_received:
			self.em_stop_pressed = 'NO MSG'
		
		
		
		
		## TODO: Check that all the parameters (targets, topic, etc..) are set properly for each component
		## TODO: Check that at least one of the components (base, actuator, sensor) is received from param server
		## TODO: Improve cb_function selection in check_msg function
		## TODO: Result-file path given as a parameter
		## TODO: Init base
		## TODO: Save log for 'Target position out of error range'
		## TODO: Save log for 'Could not move base to Target/Home'
		## TODO: Time limit for move_base function excecuting sss.stop() command after predefined limit
		## TODO: Don't try to move component, if initialization wasn't successful
		## TODO: Add not rospy.is_shutdown() condition to every loop
		## TODO: Test summary (how many tests performed per component and how many fails etc..)
		
		
		
		
		# Save test info to results file
		self.log_file.write('Component test %s' %(time.strftime('%d.%m.%Y')))
		self.log_file.write('\n\nTested components: \n')
		if self.base_goals != None:
			self.log_file.write('  base\n')
		try:
			for actuator in self.actuators:
				self.log_file.write('  ' + actuator['name'] + '\n')
		except: pass
		self.log_file.write('\n\n')
	
	
	
	#############
	#### RUN ####
	#############
	def run(self):
		self.test_count = 0
		self.diag_count = None
		
		self.test_on = True
		self.test_trigger_server()
		
		
		set_up_components = []
		duration = rospy.Time.now() + rospy.Duration(self.test_duration)
		while duration > rospy.Time.now():
			# Init base
			if self.base_goals != None and not 'base' in set_up_components:
				init_handle = self.sss.init('base')
				set_up_components.append('base')
			# Init actuators
			for component in self.actuators:
				if not component['name'] in set_up_components and self.init_component(component):
					set_up_components.append(component['name'])
			# Move base
			self.move_base(self.base_goals)
			# Move actuators
			for component in self.actuators:
				if component['name'] in set_up_components:
					self.move_component(component, component['test_target'])
					self.move_component(component, component['default_target'])
					
			self.test_count += 1
			
		self.test_on = False
		
		self.log_file.close()
		
		rospy.sleep(5)
	
	
	
	
	
	def init_component(self, component):
		init_tries_count = 0
		init_complete = False
		
		while not init_complete:
			init_handle = self.sss.init(component['name'])
			init_tries_count += 1
			
			if self.check_msg(component['topic'], component['msg_type']):
				init_complete = True
				return True
			elif init_tries_count >= self.max_init_tries:
				init_complete = True
				message = ('Could not initialize component <<%s>>'
						   '\nerrorCode: %s'
						   %(component['name'], init_handle.get_error_code()))
				self.log_diagnostics(component['name'], message)
				return False
	
	
	
	def check_msg(self, topic, msg_type):
		self.msg_received = False
		if str(msg_type) == "JointTrajectoryControllerState": cb_func = self.cb_actuator
		elif msg_type == "LaserScan": cb_func = self.cb_scanner
		elif msg_type == "PointCloud2": cb_func = self.cb_point_cloud
		elif msg_type == "Image": cb_func = self.cb_camera
		else: raise NameError('Unknown message! No callback function defined for message type <<%s>>' %(msg_type))
		
		sub_state_topic = rospy.Subscriber(str(topic), eval(msg_type), cb_func)
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		
		while not self.msg_received and rospy.get_rostime() < abort_time:
			rospy.sleep(0.1)
		sub_state_topic.unregister()
		
		if self.msg_received:
			return True
		return False
		
	
	def move_base(self, base_goals):
		i = 0
		while True:
			next_goal = 'test_%s'%(i)
			if next_goal in base_goals:
				move_handle = self.sss.move("base", base_goals[next_goal])
				if move_handle.get_state() != 3:
					message = ('Could not move base to position <<%s>>'
							   '\nerrorCode: %s'
							   %(next_goal, move_handle.get_error_code()))
					self.log_diagnostics('base', message)
					#raise NameError('Could not move base to %s. errorCode: %s' %(next_goal, move_handle.get_error_code()))
				move_handle.wait()
				i += 1
			else: break
			
		# Back to default position
		move_handle = self.sss.move("base",base_goals['test_0'])
		if move_handle.get_state() != 3:
			message = ('Could not move base to default position <<test_0>>'
							   '\nerrorCode: %s'
							   %(move_handle.get_error_code()))
			self.log_diagnostics('base', message)
			#raise NameError('Could not move base to test_0. errorCode: %s' %(move_handle.get_error_code()))
		move_handle.wait()
		
	
	
	
	
	def move_component(self, component, target):
		move_failed = False
		
		# Move to test position
		move_handle = self.sss.move(component['name'], target)
		if move_handle.get_state() != 3:
			message = ('Could not move component <<%s>> to position <<%s>>'
					   '\nerrorCode: %s'
					   %(component['name'], target, move_handle.get_error_code()))
			self.log_diagnostics(component['name'], message)
			move_failed = True
		move_handle.wait()
		
		# Check if the target position is really reached
		if not move_failed:
			if self.check_msg(component['topic'], component['msg_type']):
				
				actual_pos = self.actuator_position
				target_pos = rospy.get_param("/script_server/" + component['name'] + "/" + target)
				target_pos = target_pos[len(target_pos) - 1]
				
				out_of_range = False
				for i in range(len(target_pos)):
					if math.fabs(target_pos[i] - actual_pos[i]) > component['error_range']:
						out_of_range = True
				if out_of_range:
					message = ('Component <<%s>> target position out of error range!'
							   '\nTarget position: <<%s>>'
							   '\nActual position: <<%s>>'
							   '\nError range: <<%s>>' 
							   %(component['name'], target, actual_pos, component['error_range']))
					self.log_diagnostics(component['name'], message)
			else:
				message = ('Could not get the actual position of component <<%s>>.'
						   '\nTarget position: <<%s>>' %(component['name'], target))
				self.log_diagnostics(component['name'], message)
				
				
				
				
		

	def log_diagnostics(self, component, message):
		if self.diag_count != self.test_count:
			self.log_file.write('======\n'
								'TEST CYCLE #%s\n'
								'======\n\n' %(self.test_count))
			
		tstamp = time.strftime('%H:%M:%S')
		rtstamp = rospy.Time.now()
		self.log_file.write('[FAIL] [%s] [%s]\n' %(tstamp, rtstamp))
		message = '  ' + message.replace("\n","\n  ")
		self.log_file.write(message + '\n')
		
		if self.em_stop_pressed:
			self.log_file.write('  EMERGENCY STOP ACTIVE\n')
			
			
		self.log_file.write('  Diagnostics:\n')
		### GET DIAGNOSTICS ###
		# Wait for the message
		self.msg_diag_received = False
		sub_diagnostics = rospy.Subscriber("/diagnostics", DiagnosticArray, self.cb_diagnostics)
		
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
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
				diagnostics = '    ' + diagnostics.replace('\n','\n    ')
				self.log_file.write(diagnostics)
			else:
				self.log_file.write('    No diagnostics found by name <<%s>>' %(name))
		else:
			self.log_file.write('    Could not subscribe to /diagnostics topic')
		self.log_file.write('\n\n\n')
		self.diag_count = self.test_count
		
		
		
		
	def handle_test_trigger(self, req):
		return self.test_on

	def test_trigger_server(self):
		#rospy.init_node('test_trigger_server')
		s = rospy.Service('test_trigger', TestTrigger, self.handle_test_trigger)
	
	
	
			
	### CALLBACKS ###
	def cb_em_stop(self, msg):
		self.em_stop_pressed = msg.emergency_button_stop
		self.em_msg_received = True
		
	def cb_actuator(self, msg):
		self.actuator_position = msg.actual.positions
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
		
	def cb_diagnostics(self, msg):
		self.diagnostics_status = msg.status
		self.msg_diag_received = True

if __name__ == "__main__":
	try:
		TEST = ComponentTestAll()
		TEST.run()
	except KeyboardInterrupt, e:
		pass
	print "exiting"
