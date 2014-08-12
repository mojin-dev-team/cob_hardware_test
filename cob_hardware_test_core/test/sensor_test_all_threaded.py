#!/usr/bin/env python
import sys
import time
import math
import thread
import threading

# ROS imports
import roslib
roslib.load_manifest('cob_hardware_test_core')
import rospy

# msg & srv imports
from sensor_msgs.msg import *
from pr2_controllers_msgs.msg import *
from diagnostic_msgs.msg import *
from cob_relayboard.msg import *
from cob_hardware_test_core.srv import TestTrigger

# care-o-bot includes
from simple_script_server import *
from dialog_client import *


class Result:
	def __init__(self):
		self.msg_received = False
		self.msg_tstamp = rospy.Time.now()


class SensorTest:
	def __init__(self):
		rospy.init_node('sensor_test_all')
		
		### PARAMETERS ###
		self.wait_time_diag = 1
		self.wait_time_sub = 2			# waiting time (in seconds) for subscribing to topic
		self.wait_time_msg = 1
		self.test_numbers = 2

		# Message types
		self.scan_msg = LaserScan
		self.kinect_msg = PointCloud2	
		self.image_msg = Image
		
		self.msg_received = False
		self.lock = threading.Lock()
		self.test_finished = False
		
		
		### GET PARAMETERS ###
		self.sensors = []
		try:
			params = rospy.get_param('/component_test/sensors')
			i=0
			for k in params.keys():
				self.sensors.append(params[k])
				self.sensors[i]['fail'] = False
				i+=1
		except:
			raise NameError('Could not find parameters under "/component_test/sensors"')
			
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
		complete_name = '%s/sensor_test_results_%s.txt' %(log_dir, time.strftime("%Y%m%d"))
		self.log_file = open(complete_name,'w')	
			
			
			
		# Prepare test-results file
		self.log_file.write('Sensor test %s' %(time.strftime('%d.%m.%Y')))
		self.log_file.write('\n\nTested sensors: \n')
		
		for sensor in self.sensors:
			self.log_file.write('  ' + sensor['name'] + '\n')
		self.log_file.write('\n\nTEST LOG:\n========')
		self.log_file.write('\n\n[INFO] [%s]'
							'\n  Test started.\n\n\n' %(time.strftime('%H:%M:%S')))
		
		
		self.starting_time = rospy.Time.now()
		
		
		
	### RUN ###
	def run(self):
		
		#rospy.sleep(5)
		##############
		if self.test_trigger():
			
			threads = []
			for sensor in self.sensors:
				
				#Reset for each cycle:
				self.message_received = False

				#self._test_hz(test_name, topic, hz, hzerror, test_duration, wait_time)
				#t = threading.Thread(self._test_hz(hz, hzerror, topic, test_duration, wait_time))
				t = threading.Thread(target=self.test_sensor, args=(sensor['name'], sensor['topic'], sensor['msg_type']))
				threads.append(t)
				t.start()
				
			#for t in threads:
			#	t.join()	
			######################################
			
			
		#duration = rospy.Time.now() + rospy.Duration(self.test_duration)
		#while duration > rospy.Time.now():
		#	rospy.sleep(0.1)
		#self.test_finished = True
		
		while not self.test_finished:
			if not self.test_trigger():
				self.test_finished = True
			rospy.sleep(0.1)
		
		
		
		rospy.sleep(5) # Let's wait to make sure that all the threads will be finished
		self.log_file.write('[INFO] [%s] [%s]'
							'\n  Test Ended.' %(time.strftime('%H:%M:%S'), rospy.Time.now()))
    
    
    
    
    
	### Sensor test ###
	def test_sensor(self, name, topic, msg_type):

		self.msg_received = False
		if str(msg_type) == "JointTrajectoryControllerState": cb_func = self.cb_actuator
		elif msg_type == "LaserScan": cb_func = self.cb_scanner
		elif msg_type == "PointCloud2": cb_func = self.cb_point_cloud
		elif msg_type == "Image": cb_func = self.cb_camera
		else: raise NameError('Unknown message! No callback function defined for message type <<%s>>' %(msg_type))
		
		result = Result()
		while not self.test_finished:
			log_sub = True
			while not self.test_finished and not result.msg_received:
				sub_state_topic = rospy.Subscriber(str(topic), eval(msg_type), cb_func, result)
				abort_time = rospy.Time.now() + rospy.Duration(self.wait_time_sub)
				
				while not result.msg_received and rospy.get_rostime() < abort_time:
					rospy.sleep(0.1)
				if not result.msg_received:
					sub_state_topic.unregister()
					if log_sub == True:
						self.write_log(name, 'Couldn\'t subscribe to topic. Continuing trying...')
						log_sub = False
			
			wtime = result.msg_tstamp + rospy.Duration(self.wait_time_msg)
			if wtime < rospy.Time.now() and not self.test_finished:
				result.msg_received = False
				self.write_log(name, 'Too much time passed since last message. Last msg received: %s Unsucbscribing and trying to subscribe again...' %result.msg_tstamp)
				sub_state_topic.unregister()
			
			#if self.starting_time + rospy.Duration(10) > rospy.Time.now():
			#	self.test_finished = True
			
			
			
	def write_log(self, name, msg):
		tstamp = time.strftime('%H:%M:%S')
		rtstamp = rospy.Time.now()
		with self.lock:
			self.log_file.write('[FAIL] [%s] [%s]'
								'\n  Sensor: %s'
								'\n  %s'
								'\n  Diagnostics:' 
								%(tstamp, rtstamp, name, msg))
			self.log_file.write('\n\n\n')
	
	
	
	
	def test_trigger(self):
		rospy.wait_for_service('test_trigger')
		test_trig = rospy.ServiceProxy('test_trigger', TestTrigger)
		resp = test_trig()
		if resp.run_test == True:
			return True
		return False
		
		
	### CALLBACKS ###

	def cb_scanner(self, msg, result):
		self.scanner_msg = msg.ranges
		self.msg_received = True
		result.msg_received = True
		result.msg_tstamp = rospy.Time.now()

	def cb_point_cloud(self, msg, result):
		self.point_cloud_msg = msg.fields
		self.msg_received = True
		result.msg_received = True
		result.msg_tstamp = rospy.Time.now()

	def cb_camera(self, msg, result):
		self.camera_msg = msg.data
		self.msg_received = True
		result.msg_received = True
		result.msg_tstamp = rospy.Time.now()
		
	def cb_diagnostics(self, msg):
		self.diagnostics_status = msg.status
		self.msg_received = True

if __name__ == "__main__":
	try:
		TEST = SensorTest()
		TEST.run()
	except KeyboardInterrupt, e:
		pass
	print "exiting"
