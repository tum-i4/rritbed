#!/usr/bin/env python
""" Generate data based on the Gaussian distribution and publish it to ROS """

import sys
import rospy


class GaussianGenerator(object):
	""" Data generation class based on the Gaussian distribution """

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		rospy.init_node("gaussian_generator", anonymous=True)


	def generate(self):
		""" Generate data until node is stopped """

		# While loop to assure that Ctrl-C can exit the app
		while not rospy.is_shutdown():
			pass


if __name__ == "__main__":
	try:
		GEN = GaussianGenerator()
		GEN.generate()
	except rospy.ROSInterruptException:
		pass
