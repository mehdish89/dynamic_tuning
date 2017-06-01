from optimizer import optimizer
import compare

class dynamic_tuner:


	_param_list = []
	_value_list = []
	_simulate = lambda _: None
	_path = ''

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


	def configure_params(param_list):
		# read through param_list to find configuration of the parameters
		# set the parameters configuration on the optimizer

		pass

	def configure_values():
		# --

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
		# return the evaluation


		pass


	def _objective(values):

		pass
	