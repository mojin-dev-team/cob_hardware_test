#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test_core')
import sys
import time
import rospy
import numpy as np
import collections

from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from simple_script_server import *
from pr2_controllers_msgs.msg import *
from dialog_client import *


class DailyMorningShow:
	
	def __init__(self):
		rospy.init_node('daily_morning_show')
		
		### PARAMETERS
		self.max_init_tries = 1		# maximum initialization tries for each component
		self.wait_time = 1		# waiting time (in seconds) before trying initialization again
		###

		self.sss = simple_script_server()	
			
		self.message_received = False
		self.all_inits_successful = True
		
		self.scan_msg = LaserScan
		self.kinect_msg = PointCloud2
		self.image_msg = Image
		self.actuator_msg = JointTrajectoryControllerState

		dict = (('init_state','NULL'), ('init_count','NULL'), ('launch_manual','NULL'), ('move_to_test','NULL'), ('move_to_home','NULL'),('recover','NULL'))
		dict = collections.OrderedDict(dict)
		
		# Uncomment for the real robot:
		#self.actuators = [["torso","home","front_down",dict.copy()],
					#["head","home","front_down",dict.copy()],
					#["sensorring","front","back",dict.copy()],
					#["arm_left","home","front",dict.copy()],
					#["arm_right","home","front",dict.copy()]]
		
		# Uncomment for simulation:
		self.actuators = [["torso","home","front_down",dict.copy()],
						["head","home","front_down",dict.copy()],
						["sensorring","front","back",dict.copy()],
						["arm_left","home","folded",dict.copy()],
						["arm_right","home","folded",dict.copy()]]
		
		
		self.scanners = [["scan_front","front","NULL"],
						["scan_left","left","NULL"],
						["scan_right","right","NULL"]]
		
		self.kinects = [["kinect_left","torso_cam3d_left","NULL"],
						["kinect_right","torso_cam3d_right","NULL"],
						["kinect_down","cam3d_down","NULL"]]
				
		self.cameras = [["cam_left","torso_cam3d_left","NULL"],
						["cam_left_flip","torso_cam3d_left/flip","NULL"],
						["cam_right","torso_cam3d_right","NULL"],
						["cam_right_flip","torso_cam3d_right/flip","NULL"],
						["cam_down","cam3d_down","NULL"]]



	def run(self):
		# Run the test
		self.init_components()
		if not self.all_inits_successful:
			self.manually_launch_component()
		self.move_components()
		self.recover_components()
		
		self.check_scanners()
		self.check_kinects()
		self.check_images()
		
		self.print_results()



	def init_components(self):
		for component in self.actuators:
			init_complete = False
			init_tries_count = 0
			while not init_complete:
				init_handle = self.sss.init(component[0])
				init_tries_count += 1
				if self.check_msg("/" + component[0] + "_controller/state", self.actuator_msg):
					#dialog_client(0, 'Successfully initialized component %s on %s. try.' %(component[0], init_tries_count))
					component[3]["init_state"] = "OK"
					component[3]["init_count"] = str(init_tries_count)
					init_complete = True
				else:
					if init_tries_count >= self.max_init_tries:
						#if not dialog_client(1, 'Could not initialize %s after %s tries. Continue the test?' %(component[0], init_tries_count)):
						#	raise NameError('could not initialize %s after %s tries.' %(component[0], init_tries_count))
						component[3]["init_state"] = "FAIL"
						component[3]["init_count"] = str(init_tries_count)
						self.all_inits_successful = False
						init_complete = True
					else:
						init_complete = False		


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

	
	
	def manually_launch_component(self):
		s = "Please try to launch the following components manually now: \n"
		for init_state in self.actuators:
			if init_state[3]["init_state"] == "FAIL":
				s += "- %s \n" %(init_state[0])
		dialog_client(0, s + '\nPress OK after launching the components manually.')
		
		s_manual_ok = ""
		s_manual_fail = ""
		for component in self.actuators:
			if component[3]["init_state"] == "FAIL" and self.check_msg(component[0], self.actuator_msg):
				s_manual_ok += "- %s \n" %(component[0])
				component[3]["launch_manual"] == "OK"
			elif component[3]["init_state"] == "FAIL" and not self.check_msg(component[0], self.actuator_msg):
				s_manual_fail += "- %s \n" %(component[0])
				component[3]["launch_manual"] == "FAIL"
		dialog_client(0, ('Following components were successfully initialized manually: \n%s'%(s_manual_ok) if s_manual_ok!='' else '') + ('\nFollowing components could not be launched manually: \n%s'%(s_manual_fail) if s_manual_fail!='' else ''))
	
	
	
	def move_components(self):
		for component in self.actuators:
			if component[3]["init_state"] == "OK":
				# Move to test position
				move_handle = self.sss.move(component[0], component[2])
				if move_handle.get_state() != 3:
					component[3]["move_to_test"] = "FAIL"
					#raise NameError('Could not move component %s. Component state: %s' %(component[0], move_handle.get_state()))
				#self.check_target_reached(component[2])
				else: component[3]["move_to_test"] = "OK"

				# Move back to home position
				move_handle = self.sss.move(component[0], component[1])
				if move_handle.get_state() != 3:
					component[3]["move_to_home"] = "FAIL"
					#raise NameError('Could not move component %s. Component state: %s' %(component[0], move_handle.get_state()))
				#self.check_target_reached(component[1])
				else: component[3]["move_to_home"] = "OK"
			else:
				component[3]["move_to_test"] = "NOT_INIT"
				component[3]["move_to_home"] = "NOT_INIT"
			
							
	#def check_target_reached(self, target):
	
	
	def recover_components(self):
		dialog_client(0, 'Please activate and release the EM Stop. Press "OK" AFTER releasing the EM Stop.')
		rospy.sleep(3.0)
		for component in self.actuators:
			recover_handle = self.sss.recover(component[0])
			if recover_handle.get_error_code() != 0:
				#dialog_client(0, 'Could not recover component %s' %(component[0]))
				component[3]["recover"] = "FAIL"
			else:
				component[3]["recover"] = "OK"

####################
### Sensor tests ###
####################
	def check_scanners(self):
		for scanner in self.scanners:
			if self.check_msg("/scan_%s" %(scanner[1]), self.scan_msg):
				scanner[2] = "OK"
			else:
				scanner[2] = "NO_MSG"

	def check_kinects(self):
		for kinect in self.kinects:
			if self.check_msg("/%s/depth_registered/points" %(kinect[1]), self.kinect_msg):
				kinect[2] = "OK"
			else:
				kinect[2] = "NO_MSG"
				
	def check_images(self):
		for camera in self.cameras:
			if self.check_msg("/%s/rgb/image_raw" %(camera[1]), self.image_msg):
				camera[2] = "OK"
			else:
				camera[2] = "NO_MSG"
######################
### /Sensor tests  ###
######################	
	
	

	def print_results(self):
		# Prepare actuator results
		actuator_results = np.chararray((len(self.actuators)+3, len(self.actuators)+1), itemsize=15)
		actuator_results.fill('')
		for i, component in enumerate(self.actuators):
			actuator_results[0,i+1] = str(component[0])
			j = 0
			for key, value in component[3].iteritems():
				actuator_results[j+1,0] = str(key)
				actuator_results[j+1,i+1] = str(value)
				j += 1
		
		# Prepare sensor results
		sensor_results = np.array(self.scanners + self.kinects + self.cameras)
		sensor_results = np.delete(sensor_results, 1, axis=1)
		empty_fill = np.chararray((sensor_results.shape[0], 4), itemsize=10)
		empty_fill.fill('')
		sensor_results = np.concatenate((sensor_results, empty_fill), axis=1)
		
		# Output results into an excel-compatible file
		save_results = np.concatenate((actuator_results, sensor_results), axis=0)
		save_results[0,0] = time.strftime("%d.%m.%Y")
		dialog_client(0, '%s' %(save_results))
		output_dir = "/home/nhg-tl/Documents/results/daily_morning_show_results_%s.tsv" %(time.strftime("%Y%m%d"))
		np.savetxt(output_dir, save_results, delimiter="\t", fmt="%s")
		
		dialog_client(0, 'Test complete! \n\nReview results from the file "%s"' %(output_dir))
		

	def callback_state(self, msg):
		#self.actual_pos = msg.actual.positions
		self.message_received = True

if __name__ == '__main__':
	
	try:
		TEST = DailyMorningShow()
		TEST.run()
	except KeyboardInterrupt, e:
		pass
	print "exiting"
