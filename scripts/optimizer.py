# CMA-ES implementation here as a subclass of optimizer

import cma
import sys
from timeout import timeout


class optimizer:

	_params_desc = {}
	_eval_callback = lambda _:None
	_timeout = 60

	# SIGMA = 0.005
	SIGMA = 0.25



	def set_params_desc(self, desc):
		# update the configuration of the parameters e.g. min, max, default values.
		self._params_desc = desc
		pass

	def set_eval_callback(self, func):
		# set the objective value that the optimizer uses for evaluation
		self._eval_callback = func
		pass

	def set_params_config(self, config):
		for key, value in config.items():
			self._params_desc[key]['default'] = value
		pass

	def __init__(self, desc, eval_callback, timeout = 60):
		# initializer
		# set the params configs and the objective function
		self.set_params_desc(desc)
		self.set_eval_callback(eval_callback)
		self.set_timeout(timeout)
		pass

	def set_timeout(self, timeout):
		self._timeout = timeout
		pass

	def optimize(self):

		default_values =  [ self.normalize(param_desc) for _, param_desc in self._params_desc.items() ]
		print default_values

		# do the normalization and interfacing

		# return self.eval(default_values)

		res = cma.fmin(self.eval, default_values, self.SIGMA)
		return res


	def extract_config(self, values):
		params = self._params_desc.keys()

		config = {}
		for i in range(len(params)):
			param = params[i]
			desc = self._params_desc[param]
			config[param] = self.denormalize(values[i], desc["min"], desc["max"])			

		return config

	def normalize(self, param_desc):
		# default_values =  [ param_desc['default'] for _, param_desc in params_desc.items() ]
		value_range = param_desc['max'] - param_desc['min']
		value = (param_desc['default'] - param_desc['min']) / value_range
		return value

	def denormalize(self, value, vmin, vmax):
		value_range = vmax - vmin
		nvalue = value * value_range + vmin
		return nvalue

		

	def eval(self, values):

		print "raw values are: {0}".format(values)

		if any(val>1 or val<0 for val in values):
			print("OUT OF BOUND")
			return 10000000000000

		try:
			config = self.extract_config(values)
			print "extracted configs are: {0}".format(config)

			print "timeout is: {0}".format(self._timeout)			
			@timeout(self._timeout)
			def timeout_wrapper():
				return self._eval_callback(config)
			res = timeout_wrapper()
			
			print "result is: {0}".format(res)
			return res

		except: # TypeError: # catch *all* exceptions
			e = sys.exc_info()[0]
			print("Something went wrong. Error: %s" % e)		
			print("returning infinity")
			# raise e
			return 10000000000000
