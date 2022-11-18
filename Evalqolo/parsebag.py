#!/usr/bin/env python2
import sys
import os
import tf2_py
import rospy
import rosbag
import numpy as np
import matplotlib.pyplot as plt

plotRobot = True
plotAllTracks = True
trackEndDistMin = 0.0

class BagTracker:
	def __init__(self, bag):
		min_id = np.inf
		max_id = -np.inf
		for topic, msg, t in bag.read_messages(topics=['/rwth_tracker/tracked_persons']):
			for track in msg.tracks:
				if track.track_id > max_id:
					max_id = track.track_id
				if track.track_id < min_id:
					min_id = track.track_id

		n = max_id - min_id + 1
		m = bag.get_message_count('/rwth_tracker/tracked_persons')

		if m == 0:
			self.empty = True
			return
		else:
			self.empty = False


		self.X = np.ones((m, n))*np.nan
		self.Y = np.ones((m, n))*np.nan
		self.T = np.zeros((m))

		#self.iMin = -np.ones((n), dtype=int)
		#self.iMax = -np.ones((n), dtype=int)
		XY_start = np.ones((2, n))*np.nan
		XY_end = np.ones((2, n))*np.nan

		i = 0
		for topic, msg, t in bag.read_messages(topics=['/rwth_tracker/tracked_persons']):
			self.T[i] = t.to_sec()
			for track in msg.tracks:
				j = track.track_id - min_id
				self.X[i, j] = track.pose.pose.position.x
				self.Y[i, j] = track.pose.pose.position.y

				if np.isnan(XY_start[0, j]):
					XY_start[:, j] = [self.X[i, j], self.Y[i, j]]

				XY_end[:, j] = [self.X[i, j], self.Y[i, j]]
				
				#if self.iMin[j] == -1:
				#	self.iMin[j] = i
				#self.iMax[j] = i
			i += 1

		thresh = trackEndDistMin*trackEndDistMin
		jj = np.where(np.logical_not(
			np.greater(np.sum((XY_start - XY_end)*(XY_start - XY_end), 0), thresh)))

		self.X = np.delete(self.X, jj, 1)
		self.Y = np.delete(self.Y, jj, 1)

	def findClosestApproach(self, Trob, XYrob, dThreshold):
		d2Min = np.ones((self.X.shape[1]))*np.inf
		iMin = np.empty((self.X.shape[1]))
		xyRobMin = np.empty((2, self.X.shape[1]))
		iTrob = 0
		for i in range(self.X.shape[0]):
			while (iTrob != Trob.shape[0] - 1) and Trob[iTrob + 1] < self.T[i]:
				iTrob += 1
			if self.T[i] < Trob[iTrob]:
				continue
			if iTrob == Trob.shape[0] - 1:
				break
			w_i = (Trob[iTrob + 1] - self.T[i])/(Trob[iTrob + 1] - Trob[iTrob])
			xy_i = w_i*XYrob[iTrob, :] + (1 - w_i)*XYrob[iTrob + 1, :]
			d2 = ((self.X[i, :] - xy_i[0])*(self.X[i, :] - xy_i[0]) +
					(self.Y[i, :] - xy_i[1])*(self.Y[i, :] - xy_i[1]))
			ind = d2 < d2Min
			d2Min[ind] = d2[ind]
			iMin[ind] = i
			xyRobMin[:, ind] = np.reshape(xy_i, (2, 1))
			
		ind = np.less(d2Min, dThreshold*dThreshold)
		X = self.X[:, ind]
		Y = self.Y[:, ind]
		dMin = np.sqrt(d2Min[ind])
		iMin = iMin[ind]
		return (X, Y, dMin, iMin, xyRobMin[:, ind])

def extractTfs(bag):
	tf_buffer = tf2_py.BufferCore(rospy.Duration(1000000.0))

	for topic, msg, t in bag.read_messages(topics=['/tf', '/tf_static']):
		for tf_msg_entry in msg.transforms:
			if topic == '/tf_static':
				tf_buffer.set_transform_static(tf_msg_entry, "default_authority")
			else:
				tf_buffer.set_transform(tf_msg_entry, "default_authority")
	return tf_buffer

def bagToNpy(bag_file_path, output_directory):
	global plotAllTracks
	
	directory, file_name_base = os.path.split(bag_file_path)

	print ("Importing the rosbag " + file_name_base + " ...")

	bag = rosbag.Bag(bag_file_path)

	print ("Extracting TFs ...")

	tf_buffer = extractTfs(bag)

	print ("Plotting the robot's trajectory ...")

	XYrob = np.empty((bag.get_message_count('/t265/odom/sample'), 2))
	altXYrob = np.empty((bag.get_message_count('/t265/odom/sample'), 2))
	Trob = np.empty((bag.get_message_count('/t265/odom/sample')))
	i = 0
	j = 0
	for topic, msg, t in bag.read_messages(topics=['/t265/odom/sample']):
		Trob[i] = t.to_sec()
		altXYrob[i, :] = [msg.pose.pose.position.x, msg.pose.pose.position.y]
		try:
			tf_stamped = tf_buffer.lookup_transform_core("tf_qolo_world", "tf_qolo", t)
			XYrob[i, :] = [tf_stamped.transform.translation.x, tf_stamped.transform.translation.y]
		except tf2_py.ExtrapolationException:
			j += 1
			XYrob[i, :] = np.ones((2))*np.nan
		i += 1

	print (i)
	print (j)

	callXYrob = np.empty((bag.get_message_count('/oc_call'), 2))
	i = 0
	for topic, msg, t in bag.read_messages(topics=['/oc_call']):
		#n_ped = (len(msg.s) - 9)/6
		#for k in range(n_ped):
		#	msg.s[(k*2):(k*2 + 1)]
		callXYrob[i, :] = msg.x[0:2]
		i += 1

	refXrob = np.empty((13, bag.get_message_count('/oc_reply')))
	refYrob = np.empty((13, bag.get_message_count('/oc_reply')))
	i = 0
	for topic, msg, t in bag.read_messages(topics=['/oc_reply']):
		refXrob[:, i] = msg.Px
		refYrob[:, i] = msg.Py
		i += 1

	bt = BagTracker(bag)

	if plotRobot:
		plt.plot(XYrob[:, 0], XYrob[:, 1], "r")
		plt.plot(altXYrob[:, 0], altXYrob[:, 1], "k")
		plt.plot(callXYrob[:, 0], callXYrob[:, 1], "go")
		plt.plot(refXrob, refYrob, color=(0.5, 1.0, 0.5, 1.0))
	if bt.empty:
		plotAllTracks = False
	if plotAllTracks:
		plt.plot(bt.X, bt.Y) #, color=(0,0,0,1))
	if plotRobot or plotAllTracks:
		plt.gca().set_aspect(1)
		plt.show()

	if not bt.empty:
		(X_cl, Y_cl, d_cl, i_cl, xy_rob_cl) = bt.findClosestApproach(Trob, altXYrob, 2.0)

	if True:
		fig = plt.figure()
		ax = fig.add_subplot(111)
		for j in range(X_cl.shape[1]):
			line, = ax.plot(X_cl[:, j], Y_cl[:, j])
			plt.plot(xy_rob_cl[0, j], xy_rob_cl[1, j], marker="o", color = line.get_color())
		ax.set_aspect(1)
		plt.show()

if __name__ == '__main__':
	if not len(sys.argv) == 3:
		print ("Usage: python2 parsebag.py <bag_file_path> <output_directory>")
		quit()
	bagToNpy(sys.argv[1], sys.argv[2])