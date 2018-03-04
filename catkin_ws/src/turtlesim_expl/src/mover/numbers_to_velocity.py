#!/usr/bin/env python
"""
Pipe to convert some published number values to a velocity output for turtles
Arguments:
</input/topic> <topic type> </output/namespace/> [turtle name]
input topic:      Complete rostopic path
topic type:       float (Float32), int (Int32)
output namespace: Namespace in which the turtle is registered
turtle name:      (optionally) The name of the turtle (default: "turtle1")
"""

import argparse
import os
import sys
import rospy
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist

import move_helper


class NumbersToVelocity(object):
	""" Number input to turtle velocity output pipe """

	def __init__(self, args):
		""" Ctor """

		object.__init__(self)

		self._input_topic = args.input_topic
		self._topic_type = self._get_topic_type_from_str(args.topic_type)
		self._output_namespace = self._format_output_namespace(args.output_namespace)
		self._turtle_name = args.turtle_name

		self._velocity_publisher = None

		self._just_chose_speed = True
		self._turtle_walks = True


	def _get_topic_type_from_str(self, topic_type_str):
		""" Set the topic type based on the given string. """

		if topic_type_str == "float":
			return Float32
		elif topic_type_str == "int":
			return Int32
		else:
			raise ValueError("Invalid topic type given: {}\nExpected: float or int".format(topic_type_str))


	def _format_output_namespace(self, output_namespace):
		""" Format the given output namespace. """

		if not output_namespace.endswith(os.sep):
			return output_namespace + os.sep
		else:
			return output_namespace


	def activate(self):
		""" Initialise the node, activate the topics and spin """

		rospy.init_node("numbers_to_velocity_pipe", anonymous=True)

		rospy.Subscriber(self._input_topic, self._topic_type, self._pipe)

		_output = self._output_namespace + self._turtle_name + "/cmd_vel"
		self._velocity_publisher = rospy.Publisher(
			_output, Twist, queue_size=10)

		rospy.loginfo(
			"Started piping from %s to %s, with type %s",
			self._input_topic,
			_output,
			self._topic_type.__name__)

		# Keep this node from exiting until it's stopped
		rospy.spin()


	def _pipe(self, data):
		""" Pipe the given data """

		vel_msg = move_helper.get_zero_twist()

		# Decide direction or speed?
		if self._just_chose_speed:
			self._turtle_walks = data.data > 0
			self._just_chose_speed = False
			return

		if self._turtle_walks:
			vel_msg.linear.x = data.data
		else:
			vel_msg.angular.z = data.data

		self._velocity_publisher.publish(vel_msg)



if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser(prog="num2vel")

		# </input/topic> <topic type> </output/namespace/> [turtle name]

		PARSER.add_argument("input_topic", help="Complete rostopic path, e.g. /input/topic")
		PARSER.add_argument("topic_type", choices=["float", "int"])
		PARSER.add_argument("output_namespace", help="Namespace in which the turtle is registered")
		# nargs="?" == optional
		PARSER.add_argument("turtle_name", nargs="?", default="turtle1")

		# Pass filtered args to parser (remove remapping arguments and delete program name)
		ARGS = PARSER.parse_args(rospy.myargv(sys.argv)[1:])

		PIPE = NumbersToVelocity(ARGS)
		PIPE.activate()
	except rospy.ROSInterruptException:
		pass
