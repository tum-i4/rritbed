#!/usr/bin/env python
"""
Generate data based on a distribution and publish it to ROS
Possible arguments:
gauss [loc] [scale] : Gaussian distribution
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

	# pylint: disable-msg=E1101
	_generators = {
		_gaussian_str : DistributionGenerator(np.random.normal, _gaussian_str),
		_gumbel_str : DistributionGenerator(np.random.gumbel, _gumbel_str),
		_laplace_str : DistributionGenerator(np.random.laplace, _laplace_str)
	}


	def __init__(self):
		""" Ctor """

		object.__init__(self)

		# Remove remapping arguments
		args = rospy.myargv(sys.argv)

		# Delete program name from arguments
		del args[0]

		if len(args) is 0:
			raise("Sub-routine name not given")

		try:
			generator = self._generators[args[0]]
		except KeyError:
			raise("Could not find specified sub-routine " + args[0])

		self._sub_routine = args[0]

		# Delete sub-routine name from arguments
		del args[0]

		rospy.init_node(generator.name, anonymous=True)
		self._publisher = rospy.Publisher(generator.name, Float32, generator.queue_size)

		# Remaining in args are the arguments given to the sub-routine
		# pylint: disable-msg=W1202
		if len(args) is not generator.args_count:
			rospy.loginfo("Initialising with default values {}".format(generator.default_values))
			self.values = generator.default_values
			return

		rospy.loginfo("Initialising with values {}".format(args))
		self._values = [float(x) for x in args]


	def run(self):
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
		PUB.run()
	except rospy.ROSInterruptException:
		pass
