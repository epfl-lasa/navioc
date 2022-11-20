#!/usr/bin/env python2
#import sys
#import os
#import tf2_py
import rospy
import rosbag
import numpy as np
#import matplotlib.pyplot as plt

def findids(bag):
	ids = np.array([np.iinfo(np.int32).max], dtype=np.int32)
	for topic, msg, t in bag.read_messages(topics=['/rwth_tracker/tracked_persons']):
		ids_msg = np.array([track.track_id for track in msg.tracks], dtype=np.int32)
		ii = np.searchsorted(ids, ids_msg)
		ind_new = np.not_equal(ids[ii], ids_msg)
		ids = np.insert(ids, ii[ind_new], ids_msg[ind_new])
	return ids[:-1]

class TrackedPersons:
	def __init__(self, bag, t0):
		self.ids = findids(bag)
		lookup = dict.fromkeys(self.ids)
		for j, key in enumerate(self.ids):
			lookup[key] = j

		self.n_tracks = len(self.ids)
		self.n_msg = bag.get_message_count('/rwth_tracker/tracked_persons')

		self.t = np.empty((self.n_msg))
		self.states = np.empty((self.n_msg, self.n_tracks, 4)) # [Px, Py, Vx, Vy]
		self.isdef = np.zeros((self.n_msg, self.n_tracks), dtype=bool) # indicates when the track is defined

		for i, (topic, msg, t) in enumerate(bag.read_messages(topics=['/rwth_tracker/tracked_persons'])):
			self.t[i] = (t - t0).to_sec()
			for track in msg.tracks:
				j = lookup[track.track_id]
				self.isdef[i, j] = True
				self.states[i, j, :] = [track.pose.pose.position.x, track.pose.pose.position.y,
					track.twist.twist.linear.x, track.twist.twist.linear.y]

		#print (self.t[0], self.t[-1], self.n_tracks)

class OCreply:
	def __init__(self, bag, t0, T_horizon=12):
		n_msg = bag.get_message_count('/oc_reply')

		self.t = np.empty((n_msg))
		self.states = np.empty((n_msg, T_horizon + 1, 4))
		self.u = np.empty((n_msg, T_horizon, 2))
		for i, (topic, msg, t) in enumerate(bag.read_messages(topics=['/oc_reply'])):
			self.t[i] = (t - t0).to_sec()
			self.states[i, :, 0] = msg.Px
			self.states[i, :, 1] = msg.Py
			self.states[i, :, 2] = msg.Vx
			self.states[i, :, 3] = msg.Vy
			self.u[i, :, 0] = msg.Ax
			self.u[i, :, 1] = msg.Ay

class OCcall:
	def __init__(self, bag, t0, T_horizon=12):
		n_msg = bag.get_message_count('/oc_call')

		self.t = np.empty((n_msg))
		self.v_des = np.empty((n_msg, 2)) # only robot
		self.x = np.empty((n_msg, 4)) # only robot
		for i, (topic, msg, t) in enumerate(bag.read_messages(topics=['/oc_call'])):
			self.t[i] = (t - t0).to_sec()
			Du = len(msg.v_des)
			self.x[i, :2] = msg.x[:2]
			self.x[i, 2:] = msg.x[Du:(Du + 2)]
			self.v_des[i, :] = msg.v_des[:2]

class OdomSample:
	def __init__(self, bag, t0, T_horizon=12):
		n_msg = bag.get_message_count('/t265/odom/sample')

		self.t = np.empty((n_msg))
		self.states = np.empty((n_msg, 4))
		for i, (topic, msg, t) in enumerate(bag.read_messages(topics=['/t265/odom/sample'])):
			self.t[i] = (t - t0).to_sec()
			self.states[i, :] = [msg.pose.pose.position.x, msg.pose.pose.position.y, 
				msg.twist.twist.linear.x, msg.twist.twist.linear.y]

class RemoteCommands:
	def __init__(self, bag, t0):
		n_msg = bag.get_message_count('/qolo/remote_commands')

		self.t = np.empty((n_msg))
		self.vw = np.empty((n_msg, 2))
		for i, (topic, msg, t) in enumerate(bag.read_messages(topics=['/qolo/remote_commands'])):
			self.t[i] = (t - t0).to_sec()
			self.vw[i, :] = msg.data[1:3]

class RDStoGUI:
	def __init__(self, bag, t0):
		n_msg = bag.get_message_count('/rds_to_gui')

		self.t = np.empty((n_msg))
		self.vw_orig = np.empty((n_msg, 2))
		self.vw_corr = np.empty((n_msg, 2))
		for i, (topic, msg, t) in enumerate(bag.read_messages(topics=['/rds_to_gui'])):
			self.t[i] = (t - t0).to_sec()
			self.vw_orig[i, :] = [msg.nominal_command.linear, msg.nominal_command.angular] 
			self.vw_corr[i, :] = [msg.corrected_command.linear, msg.corrected_command.angular]

# def test_msgs_reading(bag_file_path):
# 	bag = rosbag.Bag(bag_file_path)
# 	for topic, msg, t in bag.read_messages():
# 		t0 = t
# 		break
# 	tracked_persons = TrackedPersons(bag, t0)
# 	oc_reply = OCreply(bag, t0)
# 	oc_call = OCcall(bag, t0)
# 	odom_sample = OdomSample(bag, t0)
# 	remote_commands = RemoteCommands(bag, t0)
# 	rds_to_gui = RDStoGUI(bag, t0)

# 	if False:
# 		for j in range(tracked_persons.n_tracks):
# 			Pxy = tracked_persons.states[tracked_persons.isdef[:, j], j, :2]
# 			plt.plot(Pxy[:, 0], Pxy[:, 1])
# 		plt.gca().set_aspect(1)
# 		plt.show()


# if __name__ == '__main__':
# 	test_msgs_reading(sys.argv[1])