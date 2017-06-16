# from optimizer import optimizer
import sys
import numpy as np
from scipy import interpolate
from collections import defaultdict
import rospy
import dynamic_reconfigure.client
import rosbag

from numbers import Number
import math

from std_msgs.msg import String


import subprocess
import threading

import re
import yaml

from timeout import timeout
from optimizer import optimizer

import matplotlib.pyplot as plt

import os


SIM_PREFIX = "/simulation"
REAL_PREFIX = "/real"


class objective_composer(dict):

	tasks = []

	def resolve(self, a):
		if '__iter__' in dir(a):
			_a = []
			for item in a:
				_a.append(self.resolve(item))
			_a = np.array(_a)
			if(len(_a)==1):
				_a = _a[0]
		else:
			_a = self.get(a)

		return _a

	def insert(self, var, val):
		self[var] = val



	def add(self, out, *args):
		
		inp = self.resolve(args)
		inp = np.append([], inp)
		
		val = 0
		for i in inp:
			val = val + i
		
		self.insert(out, val)
		

	def push_task(self, func, out, *args):
		self.tasks.append((func, out, args))

	def clear_tasks(self):
		self.tasks = []

	def execute(self, script):
		exec(script, {}, self)

	def __call__(self, values):
		self.clear()
		self.update(values)
		for func, out, args in self.tasks:
			func(out, args)




def value_of(topic, msg, value):
	if(not value.startswith(topic)):
		return None

	tl = len(topic)
	
	attrs = value[tl:]
	attrs = re.split("/|\[|\]", attrs)

	res = msg

	for attr in attrs:
		if attr == "":
			continue

		if "__iter__" in dir(res):
			index = int(attr)
			if index >= len(res):
				return None
			res = res[index]

		else: 
			if not hasattr(res, attr):
				return None
			res = getattr(res, attr)

	return res	

def combine(bag_a, bag_b, output_bag_file = 'combined.bag'):
	print "[combining bags]"

	bag_c = rosbag.Bag(output_bag_file, 'w')
	
	for topic, msg, t in bag_a.read_messages():
		bag_c.write(topic, msg, t)

	for topic, msg, t in bag_b.read_messages():
		bag_c.write(topic, msg, t)

	bag_c.close()

	bag_c = rosbag.Bag(output_bag_file, 'r')

	return bag_c


def restamp(bag, output_bag_file = 'restamped.bag', prefix = "", start_signal = {}, end_signal = {}):
	print "[restamping {0}]".format(prefix)

	state = -1
	if not start_signal:
		state = 0


	output = rosbag.Bag(output_bag_file, 'w')

	
	zero = rospy.Duration(0.)


	def check_signal(topic, msg, signal):

		for key, val in signal.items():
			new_val = value_of(topic, msg, key)
			if (val!=None and val == new_val) or (val == None and new_val != None):
				# print("happened with", topic, msg, signal)
				return True
		# print("didnt happen with", topic, msg, signal)
		return False


	try:
		for topic, msg, t in bag.read_messages():
			# print(zero)
			if(zero == rospy.Duration(0.)):
				zero = t - rospy.Duration(0.1) # correct the delay
			if state == -1:
				if check_signal(topic, msg, start_signal):
					zero = t - rospy.Duration(0.1) # correct the delay
					state = 0
					pass # START TO RESTAMP

			if state == 0:

				### RESTAMPING
				# print("#####", topic, msg, (t-zero).to_sec())
				new_topic = "".join([prefix, topic])
				# output.write(new_topic, msg, t-zero)
				output.write(new_topic, msg, t-zero)

				if check_signal(topic, msg, end_signal):
					state = 1
					pass # STOP RESTAMP

			if state == 1:
				break
	finally:
		output.close()

	output = rosbag.Bag(output_bag_file, 'r')

	return output


def read_value_vector(bag, value):
	topics = bag.get_type_and_topic_info()[1].keys()
	for topic in topics:
		if value.startswith(topic):
			data = []
			stamps = []
			for _, msg, t  in bag.read_messages([topic]):
				val = value_of(topic, msg, value)
				data.append(val)
				# stamps.append(msg.header.stamp.to_sec() if msg._has_header else t.to_sec())
				stamps.append(t.to_sec())

			data = np.array(data)
			stamps = np.array(stamps)
			return data, stamps
	return np.array([]), np.array([])


def interpobj(t, y):

	# TODO: handle the cases where its far from any point
	# TODO: make it faster with binary search, dict or sth

	t = np.append([float("-inf")], t)
	# y = np.append([None], y)

	def f(t_new):

		if '__iter__' not in dir(t_new):
			return y[max(i for i in range(len(t)) if t[i] <= t_new)]

		y_new = []
		for tt in t_new:
			index = max(i for i in range(len(t)) if t[i] <= tt)
			if index == 0:
				yy = None
			else:
				yy = y[index-1]
			y_new.append(yy)

		y_new = np.array(y_new)	
		return y_new

	return f


def play_bag(bag_file, src, dst):
	# play the messages from the topic <src> of the bag on the topic <dst>	
	# cmd = "rosbag play -d 2 --topics {0} --bags={1} {2}:={3}".format(src, bag_file, src, dst)

	cmd = ["rosbag",  "play", "-d 2", 
			"--topics", src, 
			"--bags=", bag_file,
			"{0}:={1}".format(src, dst)]

	FNULL = open(os.devnull, 'w')	
	return subprocess.Popen(cmd, stdout=FNULL, stderr=subprocess.STDOUT)

def record_bag(output_bag, topics = [], duration = 10):
	topics = " ".join(topics)
	if topics == "":
		topics = "--all"

	FNULL = open(os.devnull, 'w')
	return subprocess.Popen(["rosbag", "record", topics,
							"--duration={0}".format(duration),
							"--output-name={0}".format(output_bag) ], 
							stdout=FNULL, stderr=subprocess.STDOUT)


def get_bag_info(bag):
	if isinstance(bag, str):
		bag = rosbag.Bag(bag, 'r')
	return yaml.load(bag._get_yaml_info())

def start_simulation(real_bag_file, sim_bag_file, src_topic, dst_topic, start_signal={}, end_signal={}):
	real_bag = rosbag.Bag(real_bag_file, 'r')
	info = get_bag_info(real_bag)
	
	# @timeout(info["duration"] + 3 + 5)
	# @timeout(3)

	duration = info["duration"] + 3
	_timeout = int(duration + 2) 

	rec_proc = record_bag(sim_bag_file, duration = duration)
	play_proc = play_bag(real_bag_file, src_topic, dst_topic)

	try:
		@timeout(_timeout)
		def wait():
			is_playing = True
			is_recording = True

			while is_playing or is_recording:
				if rec_proc.poll() is not None and is_recording:
					is_recording = False
					print("[finished recording]")
				# play_proc.wait()
				if play_proc.poll() is not None and is_playing:
					is_playing = False
					print("[finished playing]")
				# rec_proc.wait()			
		wait()
	except:
		e = sys.exc_info()[0]
		
		print("Something went wrong. Error: %s" % e)		
		print("Killing the play/record processes")
		
		if rec_proc.poll() is None:
			rec_proc.kill()
		if play_proc.poll() is None:
			play_proc.kill()
		
		rec_proc.wait()
		play_proc.wait()

		raise e

	finally:
		if rec_proc.poll() is None:
			rec_proc.kill()
		if play_proc.poll() is None:
			play_proc.kill()

	print("[finished simulation]")


	sim_bag = rosbag.Bag(sim_bag_file, 'r')

	sim_bag = restamp(sim_bag, output_bag_file = 'sim-restamped.bag', prefix = SIM_PREFIX, start_signal = start_signal, end_signal = end_signal)
	real_bag = restamp(real_bag, output_bag_file = 'real-restamped.bag', prefix = REAL_PREFIX, start_signal = start_signal, end_signal = end_signal)

	combined_bag = combine(sim_bag, real_bag)

	return combined_bag


def resample(bag, value_list, resolution = 0.1):
	datamap = {}

	t_min = float("-inf")
	t_max = float("inf")

	for value in value_list:
		

		_, t = read_value_vector(bag, value)

		# print(t)

		t_min = max(t_min, min(t))
		t_max = min(t_max, max(t))


	print("###")
	print t_min, t_max

	# rospy.sleep(5)

	for value in value_list:
		y, t = read_value_vector(bag, value)

		print "reading {0}".format(value)
		# print (y, t)

		dt = y.dtype

		if dt == int or dt == float: # TODO: double check the correctness of type checking
			f = interpolate.interp1d(t, y, axis = 0, fill_value = "extrapolate")
		else:
			f = interpobj(t,y)



		"""
		create a new time array
		project the new samples
		type cast back
		"""
		t_new = np.arange(t_min, t_max, resolution)

		ndata = f(t_new)

		# print(t_new, ndata)

		ndata = ndata.astype(dt)

		datamap[value] = ndata
	
	return datamap



# def plot_dual(config):
# 	if '/simulation/joint_states/position[2]' in config and '/simulation/joint_states/position[2]' in config:
# 		tmp = config['/simulation/joint_states/position[2]']
# 		config['/simulation/joint_states/position[2]'] = config['/simulation/joint_states/position[0]']
# 		config['/simulation/joint_states/position[0]'] = tmp

# 	plt.clf()

# 	config_sim = { key:val for key, val in config.items() if key.startswith("/simulation") }
# 	config_real = { key:val for key, val in config.items() if key.startswith("/real") }
# 	plot(config_sim)
# 	plot(config_real)
# 	plt.draw()
# 	plt.show()


# def plot(config, style = '-'):

# 	plt.ion()

# 	index = 1
# 	size = len(config.items())

# 	keys = config.keys()
# 	if '/simulation/joint_states/position' in keys:
# 		keys.remove('/simulation/joint_states/position')

# 	if '/real/joint_states/position' in keys:
# 		keys.remove('/real/joint_states/position')
# 	keys.sort()


# 	for key in keys:

# 		val = config[key]

# 		plt.subplot(math.ceil(size/2.), 2, index)
		
# 		plt.plot(val, ls = style)#, hold=True)
# 		plt.ylabel(key)
# 		plt.xlabel('time')

# 		index+=1

# 		plt.pause(0.05)


class dynamic_tuner:


	_param_set = []
	_value_set = []

	_real_bag_file = ''
	_sim_bag_file = ''

	_src_topic = ''
	_dst_topic = ''

	_start_signal = {}
	_end_signal = {}

	_resolution = 0.1

	_objective = objective_composer()

	_params_desc = {}

	_optimizer = None


	# rospy.init_node('dyn_tuner')

	def __init__(   self, 
					param_set, 
					value_set, 
					real_bag_file, 
					sim_bag_file, 
					src_topic, 
					dst_topic, 
					start_signal = {}, 
					end_signal = {},
					resolution = 0.1,
					objective = objective_composer() ):

		# set the default objective function using given value_list
		# connect to param_list on the ddynamice reconfigure server
		# connect the value collector to the value_list
		# configure optimizer

		self._param_set = param_set
		self._value_set = value_set

		self._real_bag_file = real_bag_file
		self._sim_bag_file = sim_bag_file

		self._src_topic = src_topic
		self._dst_topic = dst_topic

		self._start_signal = start_signal
		self._end_signal = end_signal
		
		self._resolution = resolution

		self._objective = objective

		self._params_desc = self.get_params_desc(self._param_set)
		self._optimizer = optimizer(self._params_desc, self._evaluate)
		self._optimizer.set_params_config(self.read_default_config())

		pass


	def tune_params(self, config = {}):
		# call the optimizer
		# if config:
		# 	self._optimizer.set_params_config(config)
		self._optimizer.optimize()
		pass


	def update_params(self, param_list):
		# read through param_list to find configuration of the parameters
		# set the parameters configuration on the optimizer

		nparams = {}

		for param, value in param_list.items():
			if(param[-1] == '/'):
				param = param[:-1]
			index = param.rfind('/')
			node_name = param[:index]
			param_name = param[index+1:]

			if node_name not in nparams:
				nparams[node_name] = {}

			nparams[node_name].update({param_name:value})

		# print nparams

		for node, config in nparams.items():
			# print(node, config)

			service_name = node + "/set_parameters"
			# print(service_name)

			# rospy.wait_for_service(service_name)

			client = dynamic_reconfigure.client.Client(node, timeout=30)#, config_callback=callback)
			client.update_configuration(config)
			# rospy.sleep(1.)

		pass

	def read_default_config(self):
		config = {}
		for param in self._param_set:
			config[param] = rospy.get_param(param)
		return config


	def get_params_desc(self, param_list):

		nparams = {}

		for param in param_list:
			if(param[-1] == '/'):
				param = param[:-1]
			index = param.rfind('/')
			node_name = param[:index]
			param_name = param[index+1:]

			if node_name not in nparams:
				nparams[node_name] = {}

			nparams[node_name].update({param_name:param})

		desc = {}
		
		for node, config in nparams.items():
			# print(node, config)

			service_name = node + "/set_parameters"

			# print(service_name)

			# rospy.wait_for_service(service_name)
			client = dynamic_reconfigure.client.Client(node, timeout=30)#, config_callback=callback)
			desc_list = client.get_parameter_descriptions()

			for desc_item in desc_list:
				param_name = desc_item['name']
				if param_name in config:
					desc.update({config[param_name]:desc_item})

		return desc


	def _evaluate(self, config):
		# return the evaluation value for given parameters
		# 
		# update parameters
		# run simulation
		# record the bag
		# compare with ground truth
		# read the value_list
		# return the evaluation
		

#####################################################################

		self.update_params(config)
		print "[parameters are updated]"
		
		bag = start_simulation(	self._real_bag_file, 
								self._sim_bag_file, 
								self._src_topic, 
								self._dst_topic, 
								start_signal = self._start_signal, 
								end_signal = self._end_signal )


		print "[resampling]"
		value_map = resample(bag, self._value_set, resolution = self._resolution)


		# value_map = config
		# plot_dual(value_map)

		print "[calling objective function]"
		res = self._objective(value_map)
		print "[returning: {0}]".format(res)

		return res










	

	