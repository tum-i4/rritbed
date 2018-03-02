#!/usr/bin/env python
"""
Rebuilding turtlesim.cpp in Python
"""

# pylint: disable-msg=R0903; (Too few public methods)

import argparse
import sys

import rospy
from turtle_frame import TurtleFrame


class Turtlesim(object):
	""" The turtlesim """

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		rospy.init_node("turtlesim")


	# pylint: disable-msg=R0201; (Method could be a function - only callable after __init__)
	def execute(self, draw_gui=False, intrusion=None):
		""" Run the simulation. """

		rospy.loginfo("Starting turtleframe with draw_gui=[%s] and intrusion=[%s]",
			str(draw_gui), str(intrusion))

		# pylint: disable-msg=W0612; (Unused variable - need to hold reference)
		frame = TurtleFrame(draw_gui, intrusion)

		# Block until shut down
		rospy.spin()



if __name__ == "__main__":
	PARSER = argparse.ArgumentParser(prog="tf")
	PARSER.add_argument("--draw-gui", "-g", action="store_true", dest="draw_gui")
	PARSER.add_argument("--intrusion", "-i", choices=TurtleFrame.possible_intrusion_levels)
	# Remove remapping arguments and program name
	FILTERED_ARGV = rospy.myargv(sys.argv)[1:]
	ARGS = PARSER.parse_args(FILTERED_ARGV)

	SIM = Turtlesim()
	SIM.execute(ARGS.draw_gui, ARGS.intrusion)
