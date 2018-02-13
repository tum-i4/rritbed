#!/usr/bin/env python
"""
Rebuilding turtle.cpp in Python
"""

# Currently not supported:
# - pen
# - services
#   - pen callback
#   - teleport relative
#   - teleport absolute

import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose, Color

from util.point import Point

DEFAULT_PEN_R = 0xb3
DEFAULT_PEN_G = 0xb8
DEFAULT_PEN_B = 0xff


class Turtle(object):
	""" The turtle class """

	pos = Point()

	_pose_pub = None
	_colour_pub = None

	_last_command_time = rospy.Time.from_sec(0)
	_x_vel = 0.0
	_y_vel = 0.0
	_x_float = 0.0
	_y_float = 0.0

	def __init__(self, name, point):
		""" Ctor """

		object.__init__(self)

		assert(isinstance(point, Point))
		self.pos = point
		self._x_float = float(point.x)
		self._y_float = float(point.y)

		rospy.init_node(name)

		rospy.Subscriber(name + "/cmd_vel", Twist, self._velocity_callback)
		self._pose_pub = rospy.Publisher(name + "/pose", Pose, queue_size=10)
		self._colour_pub = rospy.Publisher(name + "/color_sensor", Color, queue_size=10)


	def _velocity_callback(self, data):
		""" Set the velocity based on the callback. """

		self._last_command_time = rospy.Time.now()
		self._x_vel = data.linear.x
		self._y_vel = data.linear.y


	def update(self, dtime, background, canvas_width, canvas_height):
		"""
		Update the turtle state and position.
		canvas_width: Expected to be max index of the x side
		canvas_height: Like canvas_width, but for y
		"""

		old_pos = Point.copy(self.pos)

		# Movement commands are only valid for one second
		if (rospy.Time.now() - self._last_command_time > rospy.Duration(1.0)):
			self._x_vel = 0.0
			self._y_vel = 0.0

		self._x_float += self._x_vel * dtime
		self._y_float += self._y_vel * dtime

		self.pos = Point(math.floor(self._x_float), math.floor(self._y_float))

		# Clamp to screen size
		if (self.pos.x < 0 or self.pos.x > canvas_width
			or self.pos.y < 0 or self.pos.y > canvas_height):
			rospy.logdebug("I hit the wall! (Clamping from [x=%f, y=%f])", self.pos.x, self.pos.y)


		self.pos.update(
			x=min(max(self.pos.x, 0), canvas_width - 1),
			y=min(max(self.pos.y, 0), canvas_height - 1))

		# Publish pose of the turtle
		pose = Pose()
		pose.x = self.pos.x
		pose.y = self.pos.y
		self._pose_pub.publish(pose)

		# Figure out (and publish) the color underneath the turtle
		colour = Color()
		rgb = background[self.pos.x][self.pos.y]
		colour.r = rgb.r
		colour.g = rgb.g
		colour.b = rgb.b
		self._colour_pub.publish(colour)

		rospy.logdebug("[%s]: pos_x: %f pos_y: %f", rospy.get_namespace(), self.pos.x, self.pos.y)

		return self.pos != old_pos



if __name__ == "__main__":
	T = Turtle("turtle1", Point())
	rospy.spin()
