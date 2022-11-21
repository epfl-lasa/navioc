from importbag import BagLoader
from exportbag import export
from os import listdir
from os.path import isfile, join, abspath

def process_dir(rootdir):
	bagdir = join(rootdir, 'bags')
	matdir = join(rootdir, 'matfiles')
	bagfiles = [f for f in listdir(bagdir) if isfile(join(bagdir, f))]
	for f in bagfiles:
		bag_file_path = join(bagdir, f)
		#print bag_file_path
		bag = BagLoader(bag_file_path)
		mat_file_path = abspath(join(matdir, bag_file_path[-23:-3] + 'mat'))
		export(bag, mat_file_path)
		#print mat_file_path

process_dir('/media/gonond/LaCieG/nov2022experiments/crowdNoon/naviocNoon')
process_dir('/media/gonond/LaCieG/nov2022experiments/crowdNoon/combNoon')
process_dir('/media/gonond/LaCieG/nov2022experiments/crowdNoon/ORCAfails')
process_dir('/media/gonond/LaCieG/nov2022experiments/crowd')
process_dir('/media/gonond/LaCieG/nov2022experiments/static_obstacle')
