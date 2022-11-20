#!/usr/bin/env python2
import rospy
import rosbag
#import numpy as np
from readmsg import TrackedPersons, OCreply, OCcall, OdomSample, RemoteCommands, RDStoGUI

class BagLoader:
	def __init__(self, bag_file_path):
		self.bag = rosbag.Bag(bag_file_path)
		for topic, msg, t in self.bag.read_messages():
			self.t0 = t
			break

		self.tracked_persons = TrackedPersons(self.bag, self.t0)
		self.oc_reply = OCreply(self.bag, self.t0)
		self.oc_call = OCcall(self.bag, self.t0)
		self.odom_sample = OdomSample(self.bag, self.t0)
		self.remote_commands = RemoteCommands(self.bag, self.t0)
		self.rds_to_gui = RDStoGUI(self.bag, self.t0)