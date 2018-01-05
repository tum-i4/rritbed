#!/usr/bin/env python
"""
Generate data based on a distribution and publish it to ROS
Possible arguments:
gaussian [loc] [scale] : Normal distribution
gumbel [loc] [scale]
laplace [loc] [scale]
logistic [loc] [scale] : Logistic distribution - scale > 0
pareto [alpha]         : Pareto distribution - alpha > 0
rayleigh [scale]       : Rayleigh distribution - scale >= 0
uniform [low] [high]   : Uniform distribution - low < high
vonmises [mu] [kappa]  : Von Mises distribution - kappa >= 0
wald [mean] [scale]    : Wald distribution - mean > 0, scale >= 0
weibull [a]            : Weibull distribution - a > 0
zipf [a]               : Zipf distribution - a > 1
"""

import sys
import numpy as np
import rospy
from std_msgs.msg import Float32

from distribution_generator import DistributionGenerator


class DistributionPublisher(object):
	""" Data generation class based on distribution """

	_publisher = None

	_sub_routine = ""

	_generator = None
	_values = []


	_gaussian_str = "gaussian"
	_gumbel_str = "gumbel"
	_laplace_str = "laplace"
	_logistic_str = "logistic"
	_pareto_str = "pareto"
	_rayleigh_str = "rayleigh"
	_uniform_str = "uniform"
	_vonmises_str = "vonmises"
	_wald_str = "wald"
	_weibull_str = "weibull"
	_zipf_str = "zipf"

	_file_based_str = "file"

	# pylint: disable-msg=E1101
	_generators = {
		# Gaussian, Gumbel, Laplace: loc and scale arbitrary
		_gaussian_str : DistributionGenerator(np.random.normal, _gaussian_str),
		_gumbel_str : DistributionGenerator(np.random.gumbel, _gumbel_str),
		_laplace_str : DistributionGenerator(np.random.laplace, _laplace_str),
		# Logistic: loc arbitrary, scale > 0
		_logistic_str : DistributionGenerator(np.random.logistic, _logistic_str),
		# Pareto: a(lpha) > 0
		_pareto_str : DistributionGenerator(np.random.pareto, _pareto_str, 1, [1.0]),
		# Rayleigh: scale > 0
		_rayleigh_str : DistributionGenerator(np.random.rayleigh, _rayleigh_str, 1, [1.0]),
		# Uniform: low < high
		_uniform_str : DistributionGenerator(np.random.uniform, _uniform_str),
		# Von Mises: mu arbitrary, kappa >= 0
		_vonmises_str : DistributionGenerator(np.random.vonmises, _vonmises_str),
		# Wald: mean > 0, scale > 0
		_wald_str : DistributionGenerator(np.random.wald, _wald_str, 2, [1.0, 1.0]),
		# Weibull: a > 0
		_weibull_str : DistributionGenerator(np.random.weibull, _weibull_str, 1, [5.0]),
		# Zipf: a > 1
		_zipf_str : DistributionGenerator(np.random.zipf, _zipf_str, 1, [2.0])
	}


	def __init__(self):
		""" Ctor """

		object.__init__(self)

		# Remove remapping arguments
		args = rospy.myargv(sys.argv)

		# Delete program name from arguments
		del args[0]

		if len(args) is 0:
			raise Exception("Sub-routine name not given")

		if args[0] is self._file_based_str:
			self.setup_reader(args.copy())
		else:
			self.setup_generator(args.copy())


	def setup_reader(self, my_args):
		"""
		Setup for file-based publishing
		my_args: Arguments trimmed to <file_name>
		"""
		if len(my_args) is 0:
			raise Exception("File name not given")
		
		pass


	def setup_generator(self, my_args):
		"""
		Setup for data generation
		my_args: Arguments trimmed to <generator> [default arg] ([default arg 2] ...)
		"""

		try:
			generator = self._generators[my_args[0]]
		except KeyError:
			raise Exception("Could not find specified sub-routine " + my_args[0])

		self._sub_routine = my_args[0]

		# Delete sub-routine name from arguments
		del my_args[0]

		rospy.init_node(generator.name, anonymous=True)
		self._publisher = rospy.Publisher(generator.name, Float32, generator.queue_size)

		# Remaining in args are the arguments given to the sub-routine
		# pylint: disable-msg=W1202
		if len(my_args) is not generator.args_count:
			self.values = generator.default_values
			rospy.loginfo("Initialising with default values {}".format(self._values))
			return

		self._values = [float(x) for x in my_args]
		rospy.loginfo("Initialising with values {}".format(self._values))


	def generate(self):
		""" Generate data until node is stopped """

		rate_limiter = rospy.Rate(self._generators[self._sub_routine].rate_in_hz)

		# While loop to assure that Ctrl-C can exit the app
		while not rospy.is_shutdown():
			next_num = self._generators[self._sub_routine].generate()
			rospy.loginfo("Value: " + str(next_num))
			self._publisher.publish(next_num)
			rate_limiter.sleep()


if __name__ == "__main__":
	try:
		PUB = DistributionPublisher()
		PUB.generate()
	except rospy.ROSInterruptException:
		pass
