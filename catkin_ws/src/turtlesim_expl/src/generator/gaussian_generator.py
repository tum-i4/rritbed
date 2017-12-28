#!/usr/bin/env python
""" Generate data based on the Gaussian distribution and publish it to ROS """

import sys
import numpy as np
import rospy
from std_msgs.msg import Float32


class GaussianGenerator(object):
	""" Data generation class based on the Gaussian distribution """

	_loc = 0.0
	_scale = 1.0

	_publisher = None


	def __init__(self):
		""" Ctor """

		object.__init__(self)

		name = "gaussian_generator"
		rospy.init_node(name, anonymous=True)
		self._publisher = rospy.Publisher(name, Float32, queue_size=10)

		args = rospy.myargv(sys.argv)

		if len(args) is not 3:
			rospy.loginfo("Initialising with default values")
			return

		try:
			loc = float(args[1])
			scale = float(args[2])
		except ValueError:
			return

		self._loc = loc
		self._scale = scale

		# pylint: disable-msg=W1202
		rospy.loginfo("Initialising with mean {} and spread {}"
			.format(self._loc, self._scale))


	def generate(self):
		""" Generate data until node is stopped """

		rate_limiter = rospy.Rate(10)

		# While loop to assure that Ctrl-C can exit the app
		while not rospy.is_shutdown():
			# pylint: disable-msg=E1101
			next_num = np.random.normal(self._loc, self._scale)
			rospy.loginfo("Last value: " + str(next_num))
			self._publisher.publish(next_num)
			rate_limiter.sleep()


if __name__ == "__main__":
	try:
		GEN = GaussianGenerator()
		GEN.generate()
	except rospy.ROSInterruptException:
		pass
