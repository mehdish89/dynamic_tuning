from optimizer import optimizer
# import compare
import numpy as np
from collections import defaultdict

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




class dynamic_tuner:


	_param_list = []
	_value_list = []
	_simulate = lambda _: None
	_path = ''


	_objective = objective()


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

		# for node, config in nparams.items():
			

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


	

	