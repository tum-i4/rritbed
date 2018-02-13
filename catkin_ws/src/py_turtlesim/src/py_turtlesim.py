#!/usr/bin/env python
"""
Rebuilding turtlesim.cpp in Python
"""

import argparse
import rospy
from turtle_frame import TurtleFrame


class Turtlesim(object):
	""" The turtlesim """

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		rospy.init_node("turtlesim")


	def execute(self, draw_gui=False):
		""" Run the simulation. """

		# pylint: disable-msg=W0612; (Unused variable - need to hold reference)
		frame = TurtleFrame(draw_gui)

		# Block until shut down
		rospy.spin()



if __name__ == "__main__":
	PARSER = argparse.ArgumentParser(prog="tf")
	PARSER.add_argument("--draw-gui", "-g", action="store_true", dest="draw_gui")
	ARGS = PARSER.parse_args()
	SIM = Turtlesim()
	SIM.execute(ARGS.draw_gui)
