#!/usr/bin/env python
"""
Rebuilding turtlesim.cpp in Python
"""

import sys
import rospy
from turtle_frame import TurtleFrame


class Turtlesim(object):
	""" The turtlesim """

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		rospy.init_node("turtlesim", argv=sys.argv)


	def execute(self):
		""" Run the simulation. """

		# pylint: disable-msg=W0612; (Unused variable - need to hold reference)
		frame = TurtleFrame()

		# Block until shut down
		rospy.spin()



if __name__ == "__main__":
	SIM = Turtlesim()
	SIM.execute()
