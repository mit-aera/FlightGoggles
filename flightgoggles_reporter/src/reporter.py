#!/usr/bin/env python

##@package flightgoggles_reporter

# See contributors for full list of authors

import rospy
import sys
import tf
import math
import numpy as np
import yaml
import signal
from std_msgs.msg import Empty

class Gate():
	## Constructor for class gate
	# @param: location: 4x3 array of corner locations
	# @param: inflation: Artificial inflation of gate to test for fly through
	def __init__(self, location, inflation):
		self.location = np.asarray(location)
		self.planeEquation = self.getPlaneOfGate()		
		
		minLoc = np.amin (self.location, axis=0)
		maxLoc = np.amax (self.location, axis=0)
		self.xmin = minLoc[0] - inflation
		self.xmax = maxLoc[0] + inflation
		self.ymin = minLoc[1] - inflation
		self.ymax = maxLoc[1] + inflation
		self.zmin = minLoc[2] - inflation
		self.zmax = maxLoc[2] + inflation


	## @brief Function to get the plane of the gate bounding box
	# @param self The object pointer
	def getPlaneOfGate(self):
		p1 = self.location[0,:]
		p2 = self.location[1,:]
		p3 = self.location[2,:]

		v1 = p3 - p1
		v2 = p2 - p1

		cp = np.cross(v1,v2)
		a,b,c = cp
		d = np.dot(cp, p3)
		return np.asarray([a,b,c,-d])

	## @brief Function to get the distance of a point from plane
	# @param self The object pointer
	# @param point The query point to calcluate distance from
	def getDistanceFromPlane(self,point):
		d = math.fabs((self.planeEquation[0] * point[0] + self.planeEquation[1] * point[1] + self.planeEquation[2] * point[2] + self.planeEquation[3]))
		e = math.sqrt(self.planeEquation[0]**2 + self.planeEquation[1]**2 + self.planeEquation[2]**2)
		return (d/e)

	## @brief Function to check if the drone is flying through a gate
	# @param self The object pointer
	# @param point The translation of the drone
	# @param tol The point to plane distance that is considered acceptable
	def isEvent(self,point, tol):
		#Check if we are inside the inflated gate
		if (point[0] < self.xmax) and (point[0] > self.xmin):
			if (point[1] < self.ymax) and (point[1] > self.ymin):
				if (point[2] < self.zmax) and (point[2] > self.zmin):
					# Compute the distance from the gate
					d = self.getDistanceFromPlane(point)
					if (d < tol):
						return True
		return False


class ReporterNode():
	## Constructor for the ReporterNode
	def __init__(self):

		nextEventId = 0
		eventTol    = 1.0

		rate = 150.
		
		# Log the event data for output
		self.eventLogData = {}
		
		# Setup a TF listener to listen for the drone pose in world
		listener = tf.TransformListener()


		# Add a trap for Ctrl+C
		signal.signal(signal.SIGINT, self.signalHandler)

		# Read the events from the challenge params
		name=rospy.get_param('/uav/challenge_name', '')
		if (name == ''):
			rospy.logerr("Challenge name could not be read")
			rospy.signal_shutdown("Challenge parameter [name] could not be read")

		rospy.loginfo("Running challenge %s", name)

		# Set a timeout to check if challenge is taking longer than allowed
		timeout = rospy.get_param('/uav/timeout', -1)
		if (timeout == -1):
			rospy.logerr("Timeout not specified!")
			rospy.signal_shutdown("Challenge parameter [timeout] could not be read")
		rospy.Timer(rospy.Duration(timeout), self.timerCallback)

		rospy.Subscriber("/uav/collision", Empty, self.collisionCallback)

		inflation = rospy.get_param("/uav/inflation", 0.1)

		# Load the required list of events from the config
		events = rospy.get_param("/uav/gate_names", '[]')
		eventTol = rospy.get_param("/uav/gate_width", 1.0)

		self.fname = rospy.get_param("/uav/results_location", "results.yaml")

		# Make gate objects to detect fly throughs later
		gates = []
		for e in events:
			loc = np.asarray(rospy.get_param("/uav/%s/location"%e, "[]"))
			if (loc.shape[0] !=4 or loc.shape[1] != 3):
				rospy.logerr("Location not specified in correct format")
				rospy.signal_shutdown("Location not specified in correct format")
			gates.append(Gate(loc, inflation))

		time_start = rospy.Time.now().to_sec()
		while not rospy.is_shutdown():
			# Get the transformation of the drone in the world
			try:
				(trans,rot) =  listener.lookupTransform('/world','/uav/imu', rospy.Time(0))
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException): 
				continue

			# check if we have passed any gate in our future objectives, if so we have to count missed gates
			for i,gate in enumerate(gates[nextEventId:]):
				if (gate.isEvent(trans,eventTol)):
					if (i > 0):
						rospy.loginfo("Skipped %d events",i)
						for it in range(i):
							self.eventLogData["Gate%d"%(nextEventId+it)]=dict(Name=events[nextEventId+it], Success="False")
					rospy.loginfo("Reached event %s at %f", events[nextEventId+i], rospy.Time.now().to_sec() - time_start)
					self.eventLogData["Gate%d"%(nextEventId+i)] = dict(Time=rospy.Time.now().to_sec() - time_start, Location=trans, Name=events[nextEventId+i], Success="True")	
					nextEventId +=(i+1)
				
				if nextEventId >= len(events):
					self.eventLogData["Result"]="Challenge Completed"
					self.writeLog()
					rospy.loginfo("Completed the challenge")
					rospy.signal_shutdown("Challenge complete")

			rospy.sleep(1./rate)

	## @brief timerCallback callback for elapsed timer to enable the reporter to timeout
	# @param self the object pointer
	# @param event timer event
	def timerCallback(self, event):
		self.eventLogData["Result"] = "Timed out"
		rospy.loginfo("The challenge timed out!")
		self.writeLog()
		rospy.signal_shutdown("Timed out!")

	## @brief writeLog function to dump the log file
	# @param self The object pointer
	def writeLog(self):
		with open(self.fname, 'w') as yaml_file:
			yaml.dump(self.eventLogData, yaml_file)

	## @brief collision callback to handle collision event
	# @param self The object pointer
	# @param data The collision message
	def collisionCallback(self, data):
		self.eventLogData["Result"]="Collision"
		rospy.loginfo("The drone collided!")
		self.writeLog()
		rospy.signal_shutdown("The drone collided")

	## @brief signalHandler to handle the reporter being interrupted by SIGINT
	def signalHandler(self, sig, frame):
		self.eventLogData["Result"]="Interrupted" 
		self.writeLog()
		rospy.loginfo("Grading was interrupted")
		rospy.signal_shutdown("Grading was interrupted")

if __name__ == '__main__':
	rospy.init_node('reporter')

	try:
		ne = ReporterNode()
	except rospy.ROSInterruptException: pass

