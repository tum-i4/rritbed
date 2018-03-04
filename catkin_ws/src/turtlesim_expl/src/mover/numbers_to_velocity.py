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

import os
import sys
import rospy
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import Twist

import move_helper


class NumbersToVelocity(object):
	""" Number input to turtle velocity output pipe """

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		self._input_topic = ""
		self._topic_type = None
		self._output_namespace = ""
		self._turtle_name = "turtle1"

		self._velocity_publisher = None

		self._just_chose_speed = True
		self._turtle_walks = True

		# Remove remapping arguments and program name
		args = rospy.myargv(sys.argv)[1:]

		if len(args) < 3 or len(args) > 4:
			raise Exception("Invalid number of arguments given: {}".format(len(args)))

		self._input_topic = args[0]

		if args[1] == "float":
			self._topic_type = Float32
		elif args[1] == "int":
			self._topic_type = Int32
		else:
			raise Exception("Invalid topic type given: {}\nExpected: float or int".format(args[1]))

		self._output_namespace = args[2]
		if not self._output_namespace.endswith(os.sep):
			self._output_namespace += os.sep

		if len(args) == 4:
			self._turtle_name = args[3]


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
		PIPE = NumbersToVelocity()
		PIPE.activate()
	except rospy.ROSInterruptException:
		pass
