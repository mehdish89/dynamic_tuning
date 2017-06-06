from optimizer import optimizer
# import compare
import numpy as np
from scipy import interpolate
from collections import defaultdict
import rospy
import dynamic_reconfigure.client
import rosbag

from numbers import Number

from std_msgs.msg import String

import subprocess
import re


class objective(dict):

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

def combine(bag_a, bag_b):
	bag_c = rosbag.Bag('combined.bag', 'w')
	
	for topic, msg, t in bag_a.read_messages():
		bag_c.write(topic, msg, t)

	for topic, msg, t in bag_b.read_messages():
		bag_c.write(topic, msg, t)

	bag_c.close()

	return bag_c


def restamp(bag, start_signal = {}, end_signal = {}):

	output = rosbag.Bag('restamped.bag', 'w')

	state = -1 + 1
	zero = rospy.Duration(0.)

	try:
		for topic, msg, t in bag.read_messages():
			if state == -1:
				for key, val in start_signal.items():
					if(val == value_of(topic, msg, key)):
						zero = t
						state = 0
						break
						pass # START TO RESTAMP

			if state == 0:

				### RESTAMPING
				print(topic, msg, t)
				output.write(topic, msg, t-zero)


				for key, val in end_signal.items():
					if(val == value_of(topic, msg, key)):
						state = 1
						break
						pass # STOP RESTAMP
			if state == 1:
				break
	finally:
		output.close()

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
				stamps.append(msg.header.stamp.to_sec() if msg._has_header else t.to_sec())

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





def resample(bag, value_list, resolution = 0.1):
	datamap = {}

	t_min = float("-inf")
	t_max = float("inf")

	for value in value_list:
		_, t = read_value_vector(bag, value)
		t_min = max(t_min, min(t))
		t_max = min(t_max, max(t))


	for value in value_list:
		y, t = read_value_vector(bag, value)

		dt = y.dtype

		if dt == int or dt == float: # TODO: double check the correctness of type checking
			f = interpolate.interp1d(t, y, axis = 0, fill_value = "extrapolate")
			# print(t,y)
		else:
			f = interpobj(t,y)



		"""
		create a new time array
		project the new samples
		type cast back
		"""
		t_new = np.arange(t_min, t_max, resolution)

		ndata = f(t_new)

		ndata = ndata.astype(dt)

		datamap[value] = ndata
	
	return datamap


def play_bag(bag_file, src, dst):
	# play the messages from the topic <src> of the bag on the topic <dst>	

	cmd = "rosbag play --topics {0} --bags={1} {2}:={3}".format(src, bag_file, src, dst)
	subprocess.check_call(cmd, shell=True)

	pass




class dynamic_tuner:


	_param_list = []
	_value_list = []
	_simulate = lambda _: None
	_path = ''


	_objective = objective()


	# rospy.init_node('dyn_tuner')


	def __init__(self, param_list, value_list, simulate, ground_truth, path = ''):
		# set the default objective function using given value_list
		# connect to param_list on the ddynamice reconfigure server
		# connect the value collector to the value_list
		# configure optimizer

		_param_list = param_list
		_value_list = value_list
		_simulate = simulate
		_path = path
		_ground_truth = ground_truth

		pass

	def tune_params(config):
		# call the optimizer

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

		print nparams

		for node, config in nparams.items():
			print(node, config)

			service_name = node + "/set_parameters"

			print(service_name)

			# rospy.wait_for_service(service_name)
    		client = dynamic_reconfigure.client.Client(node, timeout=30)#, config_callback=callback)
    		client.update_configuration(config)

		pass


		

	def resample(value_list, bag,  resolution = 0.1):


		pass

	def configure_values():
		# read through the topics/values and set them in the environment

		pass

	def configure_objective(value_list, simulate):
		# compose an objective function from the value_list
		# embed the simulator in the function
		# handle the rosbag record

		pass

	def update_values(config):
		# update the parameters value with the given configuration

		pass


	def _evaluate(params):
		# return the evaluation value for given parameters
		# 
		# update parameters
		# run simulation
		# record the bag
		# compare with ground truth
		# read the value_list
		# return the evaluation

		pass


	

	