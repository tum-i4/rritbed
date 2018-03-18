#!/usr/bin/env python
"""
Rebuilding turtlesim.cpp in Python
"""

# pylint: disable-msg=R0903; (Too few public methods)

import argparse
import os
import sys

import rospy
from turtle_frame import TurtleFrame


BASE_PATH = os.path.expanduser("~/ros")
STOP_FILE_PATH = os.path.join(BASE_PATH, "STOP")


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
			draw_gui, intrusion)

		# pylint: disable-msg=W0612; (Unused variable - need to hold reference)
		frame = TurtleFrame(draw_gui, intrusion)

		# Block until shut down and check for stop file every ten seconds
		while not rospy.is_shutdown():
			if os.path.lexists(STOP_FILE_PATH):
				rospy.logerr("!!! STOP FILE DETECTED !!! KILLED !!!")
				break

			rospy.sleep(.1)



if __name__ == "__main__":
	PARSER = argparse.ArgumentParser(prog="tf")
	PARSER.add_argument("--draw-gui", "-g", action="store_true", dest="draw_gui")
	PARSER.add_argument("--intrusion", "-i", choices=TurtleFrame.POSSIBLE_INTRUSION_LEVELS)
	# Remove remapping arguments and program name
	FILTERED_ARGV = rospy.myargv(sys.argv)[1:]
	ARGS = PARSER.parse_args(FILTERED_ARGV)

	SIM = Turtlesim()
	SIM.execute(ARGS.draw_gui, ARGS.intrusion)
