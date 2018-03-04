#!/usr/bin/env python
""" Interface for publishing to turtle """

import rospy
from geometry_msgs.msg import Twist

import move_helper
from move_strategy import MoveStrategy


class TurtleControl(object):
	""" Direct interface class for moving a turtle """

	def __init__(self, move_strategy, move_strategy_args, rate_in_hz=2, turtle_path="turtle1"):
		""" Ctor """

		object.__init__(self)

		rospy.init_node("turtle_control", anonymous=True)

		assert(issubclass(move_strategy, MoveStrategy))

		self.turtle_path = turtle_path
		self.rate_limiter = rospy.Rate(rate_in_hz)
		self.move_strategy = move_strategy(move_strategy_args)
		self.velocity_publisher = rospy.Publisher(turtle_path + "/cmd_vel", Twist, queue_size=10)


	def run(self):
		""" Generate new velocity from movement strategy until exited """

		# While loop to assure that Ctrl-C can exit the app
		while not rospy.is_shutdown():
			vel_msg = self.move_strategy.get_next()

			if vel_msg is None:
				break

			assert(isinstance(vel_msg, Twist))

			self.publish(vel_msg)
			self.rate_limiter.sleep()

		# Make sure the turtle is stopped before the program exits
		self.stop_turtle()


	def publish(self, vel_msg):
		""" Publish velocity to turtle """
		self.velocity_publisher.publish(vel_msg)


	def stop_turtle(self):
		""" Convenience method for stopping turtle (by publishing a zero velocity) """
		self.publish(move_helper.get_zero_twist())
