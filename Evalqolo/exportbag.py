#!/usr/bin/env python2
import sys
import scipy.io as sio
from importbag import BagLoader

def export(bag, mat_file_path):
	mat_file_contents = { 
		'tracked_persons'	: bag.tracked_persons,
		'oc_reply'			: bag.oc_reply,
		'oc_call'			: bag.oc_call,
		'odom_sample'		: bag.odom_sample,
		'remote_commands'	: bag.remote_commands,
		'rds_to_gui'		: bag.rds_to_gui
	}
	sio.savemat(mat_file_path, mat_file_contents)

if __name__ == '__main__':
	bag_file_path = sys.argv[1]
	mat_file_path = bag_file_path[-23:-3] + 'mat'
	export(BagLoader(bag_file_path), mat_file_path)