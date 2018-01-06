#!/usr/bin/env python
"""
Pipe to convert some published number values to a velocity output for turtles
Arguments:
</input/topic> <topic type> </output/namespace> [turtle name]
input topic:      Complete rostopic path
topic type:       float (Float32), int (Int32)
output namespace: Namespace in which the turtle is registered
turtle name:      (optionally) The name of the turtle (default: "turtle1")
"""

import os
import sys
import rospy
from std_msgs.msg import Float32, Int32

import move_helper


class NumbersToVelocity(object):

	_input_topic = ""
	_topic_type = None
	_output_namespace = ""
	_turtle_name = "turtle1"

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		# Remove remapping arguments and program name
		args = rospy.myargv(sys.argv)[1:]

		if len(args) < 3 or len(args) > 4:
			raise Exception("Invalid number of arguments given: %s", len(args))

		self._input_topic = args[0]

		if args[1] == "float":
			self._topic_type = Float32
		elif args[1] == "int":
			self._topic_type = Int32
		else:
			raise Exception("Invalid topic type given: %s\nExpected: float or int", args[1])

		self._output_namespace = args[2]

		if len(args) == 4:
			self._turtle_name = args[3]


	def process(self):
		rospy.loginfo(
			"Started piping from %s to %s/%s, with type %s",
			self._input_topic,
			self._output_namespace,
			self._turtle_name,
			self._topic_type)

		# TODO: Implement!



if __name__ == "__main__":
	try:
		PIPE = NumbersToVelocity()
		PIPE.process()
	except rospy.ROSInterruptException:
		pass
