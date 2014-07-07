#!/usr/bin/env python
import roslib
roslib.load_manifest('cob_hardware_test_core')
import sys
import time
import rospy
import numpy as np
import collections

## MESSAGES
from trajectory_msgs.msg import *
from sensor_msgs.msg import *
from cob_relayboard.msg import *
from diagnostic_msgs.msg import *
from pr2_controllers_msgs.msg import *

from simple_script_server import *
from dialog_client import *


class DailyMorningShow:
	
	def __init__(self):
		rospy.init_node('daily_morning_show')
		
		### PARAMETERS
		self.max_init_tries = 1		# maximum initialization tries for each component
		self.wait_time = 1		# waiting time (in seconds) before trying initialization again
		self.wait_time_recover = 1
		self.wait_time_diag = 3
		###

		self.sss = simple_script_server()	
			
		self.msg_received = False
		self.all_inits_successful = True
		
		self.scan_msg_type = LaserScan
		self.point_cloud_msg_type = PointCloud2
		self.image_msg_type = Image
		self.actuator_msg_type = JointTrajectoryControllerState

		self.dict = (('init_state','NULL'), 
				('init_count','NULL'), 
				('launch_manual','NULL'), 
				('move_to_test','NULL'), 
				('move_to_home','NULL'), 
				('recover','NULL'))
		dict = collections.OrderedDict(self.dict)
		
		if not rospy.get_param('~sim'):
			self.actuators = [["torso","home","home",dict.copy()],
							  ["head","home","front_down",dict.copy()],
							  ["sensorring","front","back",dict.copy()],
							  #["arm_left","home","front",dict.copy()],
							  ["arm_right","home","front",dict.copy()]]
		else:
			self.actuators = [["torso","home","front_down",dict.copy()],
							  ["head","home","front_down",dict.copy()],
							  ["sensorring","front","back",dict.copy()],
							  #["arm_left","home","folded",dict.copy()],
							  ["arm_right","home","folded",dict.copy()]]
		
		
		self.scanners = [["scan_front","front","NULL"],
						 ["scan_left","left","NULL"],
						 ["scan_right","right","NULL"]]
		
		self.point_clouds = [["point_cloud_left","torso_cam3d_left","NULL"],
							 ["point_cloud_right","torso_cam3d_right","NULL"],
							 ["point_cloud_down","cam3d_down","NULL"]]
				
		self.cameras = [["cam_left","torso_cam3d_left","NULL"],
						["cam_left_flip","torso_cam3d_left/flip","NULL"],
						["cam_right","torso_cam3d_right","NULL"],
						["cam_right_flip","torso_cam3d_right/flip","NULL"],
						["cam_down","cam3d_down","NULL"]]



	def run(self):
		# Run the test
		while self.em_stop():
			dialog_client(0, 'Release EM-stop and press ''OK''')
		self.init_components()
		if not self.all_inits_successful:
			self.manually_launch_component()
		self.move_components()
		self.recover_components()
		
		self.check_scanners()
		self.check_point_clouds()
		self.check_images()
		
		self.print_results()

	
	def em_stop(self):
		
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
	

	def init_components(self):
		for component in self.actuators:
			init_complete = False
			init_tries_count = 0
			while not init_complete:
				init_handle = self.sss.init(component[0])
				init_tries_count += 1
				if self.check_msg("/" + component[0] + "_controller/state", self.actuator_msg_type, self.cb_actuator):
					#dialog_client(0, 'Successfully initialized component %s on %s. try.' %(component[0], init_tries_count))
					component[3]["init_state"] = "OK"
					component[3]["init_count"] = str(init_tries_count)
					init_complete = True
				else:
					if init_tries_count >= self.max_init_tries:
						#if not dialog_client(1, 'Could not initialize %s after %s tries. Continue the test?' %(component[0], init_tries_count)):
						#	raise NameError('could not initialize %s after %s tries.' %(component[0], init_tries_count))
						component[3]["init_state"] = "FAIL: " + self.get_diagnostics(component)
						component[3]["init_count"] = str(init_tries_count)
						self.all_inits_successful = False
						init_complete = True
					else:
						init_complete = False		


	def check_msg(self, state_topic, msg_type, cb_function):
		self.msg_received = False
		sub_state_topic = rospy.Subscriber(state_topic, msg_type, cb_function)
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		
		while not self.msg_received and rospy.get_rostime() < abort_time:
			rospy.sleep(0.1)
			
		sub_state_topic.unregister()
		
		#dialog_client(0, 'mgs_received: %s \n\nMsg: %s' %(self.msg_received, self.actuator_msg))
		if self.msg_received:
			return True
		return False

	
	
	def manually_launch_component(self):
		s = "Please try to launch the following components manually now: \n"
		for init_state in self.actuators:
			if init_state[3]["init_state"] == "FAIL":
				s += "- %s \n" %(init_state[0])
		dialog_client(0, s + '\nPress OK after launching the components manually.')
		
		s_manual_ok = ""
		s_manual_fail = ""
		for component in self.actuators:
			if component[3]["init_state"] == "FAIL" and self.check_msg(component[0], self.actuator_msg_type, self.cb_actuator):
				s_manual_ok += "- %s \n" %(component[0])
				component[3]["launch_manual"] == "OK"
			elif component[3]["init_state"] == "FAIL" and not self.check_msg(component[0], self.actuator_msg_type, self.cb_actuator):
				s_manual_fail += "- %s \n" %(component[0])
				component[3]["launch_manual"] == "FAIL: " + self.get_diagnostics(component)
		dialog_client(0, ('Following components were successfully initialized manually: \n%s'%(s_manual_ok) if s_manual_ok!='' else '') + ('\nFollowing components could not be launched manually: \n%s'%(s_manual_fail) if s_manual_fail!='' else ''))
	
	
	
	def move_components(self):
		for component in self.actuators:
			if component[3]["init_state"] == "OK" or component[3]["launch_manual"] == "OK":
				# Move to test position
				move_handle = self.sss.move(component[0], component[2])
				if move_handle.get_state() != 3:
					component[3]["move_to_test"] = "FAIL: " + self.get_diagnostics(component)
				else:
					component[3]["move_to_test"] = "OK"
				
				# Move back to home position
				move_handle = self.sss.move(component[0], component[1])
				if move_handle.get_state() != 3:
					component[3]["move_to_home"] = "FAIL: " + self.get_diagnostics(component)
				else: component[3]["move_to_home"] = "OK"
			else:
				component[3]["move_to_test"] = "NOT_INIT"
				component[3]["move_to_home"] = "NOT_INIT"
			
							
	#def check_target_reached(self, target):
	
	
	def recover_components(self):
		dialog_client(0, 'Please activate and release the EM Stop. Press "OK" AFTER releasing the EM Stop.')
		while self.em_stop():
			dialog_client(0, 'Release the EM-stop and press ''OK''')
		
		recover_handle = []
		for component in self.actuators:
			recover_handle.append(self.sss.recover(component[0]))
		rospy.sleep(self.wait_time_recover)
		
		i = 0
		for component in self.actuators:
			if recover_handle[i].get_error_code() != 0:
				#dialog_client(0, 'Could not recover component %s' %(component[0]))
				component[3]["recover"] = "FAIL: " + self.get_diagnostics(component)
			else:
				component[3]["recover"] = "OK"
			i += 1

	####################
	### Sensor tests ###
	####################
	def check_scanners(self):
		for scanner in self.scanners:
			if self.check_msg("/scan_%s" %(scanner[1]), self.scan_msg_type, self.cb_scanner):
				scanner[2] = "OK"
			else:
				scanner[2] = "NO_MSG"

	def check_point_clouds(self):
		for point_cloud in self.point_clouds:
			if self.check_msg("/%s/depth_registered/points" %(point_cloud[1]), self.point_cloud_msg_type, self.cb_point_cloud):
				point_cloud[2] = "OK"
			else:
				point_cloud[2] = "NO_MSG"
				
	def check_images(self):
		for camera in self.cameras:
			if self.check_msg("/%s/rgb/image_raw" %(camera[1]), self.image_msg_type, self.cb_camera):
				camera[2] = "OK"
			else:
				camera[2] = "NO_MSG"
				
				
		
	#######################
	### GET DIAGNOSTICS ###
	#######################
	def get_diagnostics(self, component):
		
		# Wait for the message
		self.msg_received = False
		sub_diagnostics = rospy.Subscriber("/diagnostics_agg", DiagnosticArray, self.cb_diagnostics)
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time)
		while not self.msg_received and rospy.get_rostime() < abort_time:
			rospy.sleep(0.1)
		
		#dialog_client(0, str(self.diagnostics_status))
		
		diag_name = ""
		abort_time = rospy.Time.now() + rospy.Duration(self.wait_time_diag)
		while diag_name != ("%s_controller" %(component[0])) and rospy.get_rostime() < abort_time:
			diag_array = str(self.diagnostics_status)
			diag_array = diag_array.replace("/","")
			diag_name = (diag_array.split("name: ", 1)[1]).split("\n",1)[0]
		sub_diagnostics.unregister()
		
		###
		diag_msg = (diag_array.split("message: ", 1)[1]).split("\n",1)[0]
		dialog_client(0,"%s \n\n%s \n\n%s" %(component[0], diag_name, diag_msg))
		###
		
		if diag_name == ("%s_controller" %(component[0])):
			diag_msg = (diag_array.split("message: ", 1)[1]).split("\n",1)[0]
			#dialog_client(0, diag_name + "\n" + diag_msg)
			return '"%s"' %(diag_msg)
		return '"NO DIAGNOSTIC MSG"'
	
	

	def print_results(self):
		
		# Prepare actuator results
		actuator_results = np.chararray((len(self.actuators)+4, len(self.dict)+1), itemsize=20)
		actuator_results.fill('')
		for i, component in enumerate(self.actuators):
			actuator_results[i+2,0] = component[0]
			j=0
			for key, value in component[3].iteritems():
				actuator_results[1,j+1] = str(key)
				actuator_results[i+2,j+1] = str(value)
				j+=1
		
		# Prepare sensor results
		sensor_results = np.array(self.scanners + self.point_clouds + self.cameras)
		sensor_results = np.delete(sensor_results, 1, axis=1) # Delete the second column that contains topic names
		empty_fill = np.chararray((sensor_results.shape[0], (len(self.dict)-len(sensor_results[0])+1)), itemsize=1) # Make the sensor_result array the same width as actuator_result, so we can concatenate them
		empty_fill.fill('')
		sensor_results = np.concatenate((sensor_results, empty_fill), axis=1)
		
		# Output results into an excel-compatible file
		save_results = np.concatenate((actuator_results, sensor_results), axis=0)
		save_results[0,0] = time.strftime("%d.%m.%Y")
		dialog_client(0, '%s' %(save_results))
		get_directory = rospy.get_param('~result_dir')
		output_dir = "%s/daily_morning_show_results_%s.tsv" %(get_directory, time.strftime("%Y%m%d"))
		#output_dir = "/home/nhg-tl/Documents/results/daily_morning_show_results_%s.tsv" %(time.strftime("%Y%m%d"))
		np.savetxt(output_dir, save_results, delimiter="\t", fmt="%s")
		
		dialog_client(0, 'Test complete! \n\nReview results from the file "%s"' %(output_dir))
		
	def cb_em_stop(self, msg):
		self.em_stop_pressed = msg.emergency_button_stop
		self.em_msg_received = True
		
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
		
	def cb_diagnostics(self, msg):
		self.diagnostics_status = msg.status
		#self.diagnostics_msg = msg.message
		self.msg_received = True

if __name__ == '__main__':
	
	try:
		TEST = DailyMorningShow()
		TEST.run()
	except KeyboardInterrupt, e:
		pass
	print "exiting"
