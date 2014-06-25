#!/usr/bin/env python
import sys
import time
import math

# ROS imports
import roslib
roslib.load_manifest('cob_hardware_test_core')
import rospy

# msg imports
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from pr2_controllers_msgs.msg import *

# care-o-bot includes
from simple_script_server import *
from dialog_client import *


class ComponentTestAll:
	def __init__(self):
		rospy.init_node('component_test_all')
		
		### PARAMETERS ###
		self.max_init_tries = 1		# maximum initialization tries for each component
		self.wait_time = 1		# waiting time (in seconds) before trying initialization again
		##################
		
		# message types
		self.actuator_msg = JointTrajectoryControllerState
		self.scan_msg = LaserScan
		self.kinect_msg = PointCloud2
		self.image_msg = Image
		
		self.message_received = False
		self.sss = simple_script_server()
		
		self.actuators = [["torso","NULL"], 							
						  ["head","NULL"],
						  ["sensorring","NULL"],
						  ["arm_left","NULL"],
						  ["arm_right","NULL"]]
		
		self.scanners = [["scan_front","NULL"],					
						 ["scan_left","NULL"],
						 ["scan_right","NULL"]]
		
		self.kinects = [["kinect_left","NULL"],	
						["kinect_right","NULL"],
						["kinect_down","NULL"]]
				
		self.cameras = [["cam_left","NULL"],
						["cam_left_flip","NULL"],
						["cam_right","NULL"],
						["cam_right_flip","NULL"],
						["cam_down","NULL"]]
		
		
		### CHECK PARAMETERS ###
		# base goals
		if not rospy.has_param('~base/test_0'):
			raise NameError('Parameter base/test_0 does not exist on ROS Parameter Server. At least two goals must be set!')
		if not rospy.has_param('~base/test_1'):
			raise NameError('Parameter base/test_1 does not exist on ROS Parameter Server. At least two goals must be set!')
		# actuator goals
		for component in self.actuators:
			if not rospy.has_param('~%s/default_target' %(component[0])):
				raise NameError('Parameter %s/default_target does not exist on ROS Parameter Server' %(component[0]))
			if not rospy.has_param('~%s/test_target' %(component[0])):
				raise NameError('Parameter %s/test_target does not exist on ROS Parameter Server' %(component[0]))
		# topics
		for topic in self.actuators:
			if not rospy.has_param('~%s/topic' %(topic[0])):
				raise NameError('Parameter %s/topic does not exist on ROS Parameter Server' %(component[0]))
		########################
		
		
		
	def run(self):
		for component in self.actuators:
			self.init_components(component[0])
		dialog_client(0, 'Succesfully initialized all components!')
		self.move_base()
		for component in self.actuators:
			self.move_component(component[0])
	
	
	
	def init_components(self, component):
		init_tries_count = 0
		init_complete = False
		
		while not init_complete:
			init_handle = self.sss.init(component)
			init_tries_count += 1
			state_topic = str(rospy.get_param('~%s/topic' %(component)))
			
			if self.check_msg(state_topic, self.actuator_msg):
				init_complete = True
			elif init_tries_count >= self.max_init_tries:
				#self.ask_manual_init = True
				init_complete = True
				dialog_client(0, 'Could not initialize component %s' %(component))
				#raise NameError('Could not initialize component ' + component)
		
		
	def check_msg(self, state_topic, msg_type):
		sub_state_topic = rospy.Subscriber(state_topic, msg_type, self.callback_state)
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		
		while not self.message_received and rospy.get_rostime() < abort_time:
			rospy.sleep(0.1)
				
		if not self.message_received:
			return False
		else:
			self.message_received = False
			return True
		

	def move_base(self):
		goal_number = 0
		while rospy.has_param('~base/test_%s' %(goal_number)):
			goal = rospy.get_param('~base/test_%s' %(goal_number))
			dialog_client(0, '%s' %(goal))
			move_handle = self.sss.move("base", goal)
			if move_handle.get_state() != 3:
				raise NameError('Could not move base to "test_%s". errorCode: %s' %(goal_number, move_handle.get_error_code()))
			move_handle.wait()
			goal_number += 1
			dialog_client(0, 'Goal "test_%s" reached!' %(goal_number))
		# move back to home position
		move_handle = self.sss.move("base","test_0")
		if move_handle.get_state() != 3:
			raise NameError('Could not move base to test_0. errorCode: %s' %(component[0], move_handle.get_error_code()))
		move_handle.wait()
	

	#def move_head_torso(self, component):
		
		## move torso / head
		#tilt_amount = 1.0 # the amount of tilt during the rotation
		#for i in numpy.arange(0, 6.26, 1.25):
			#joint_3 = float(i - tilt_amount)
			#joint_1 = float(-i)
			#circle_target_point = [[joint_1, tilt_amount, joint_3]]
			#move_handle = self.sss.move(component, circle_target_point)
			#if move_handle.get_state() != 3:
				#raise NameError("Could not move %s. errorCode: %s" %(component, move_handle.get_error_code()))
			#self.check_target_reached(circle_target_point)
			
		## move back to home position
		#move_handle = self.sss.move(component, [[joint_1, 0.0, joint_3 + tilt_amount]])
		#move_handle = self.sss.move(component, [[joint_1/2, 0.0, (joint_3 + tilt_amount)/2]])
		#move_handle = self.sss.move(component, [[0.0, 0.0, 0.0]])
		#if move_handle.get_state() != 3:
				#raise NameError("Could not move %s. errorCode: %s" %(component, move_handle.get_error_code()))
		#self.check_target_reached([[0.0, 0.0, 0.0]])
	
	
	def move_component(self, component):
		test_pos = rospy.get_param('~%s/test_target' %(component))
		default_pos = rospy.get_param('~%s/default_target' %(component))
		
		# Subscribe to component's state topic to get its current position
		state_topic = str(rospy.get_param('~%s/topic' %(component)))
		sub_position = rospy.Subscriber(state_topic, self.actuator_msg, self.callback_position)
		
		# Move to test position
		move_handle = self.sss.move(component, test_pos)
		if move_handle.get_state() != 3:
			raise NameError('Could not move component %s to target position. errorCode: %s' %(component, move_handle.get_error_code()))
		self.check_target_reached(test_pos)

		# Move back to home position
		move_handle = self.sss.move(component, default_pos)
		if move_handle.get_state() != 3:
			raise NameError('Could not move component %s back to home position. errorCode: %s' %(component, move_handle.get_error_code()))
		self.check_target_reached(default_pos)
		
		sub_position.unregister() # Unsubscribe from the state topic

	## Check the difference between the actual position and commanded position
	def check_target_reached(self, target_pos):
		actual_pos = self.actual_pos
		# check if the target position is really reached
		print "actual_pos = ", actual_pos
		print "traj_endpoint = ", target_pos
		for i in range(len(target_pos)):
			self.assert_(((math.fabs(target_pos[i] - actual_pos[i])) < self.error_range), "Target position out of error_range")
		
		
		
	def callback_position(self, msg):
		self.actual_pos = msg.actual.positions

	def callback_state(self, msg):
		self.message_received = True
		
if __name__ == "__main__":
	try:
		TEST = ComponentTestAll()
		TEST.run()
	except KeyboardInterrupt, e:
		pass
	print "exiting"
