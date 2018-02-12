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

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose, Color

from util.rgb import Rgb
from util.point import Point

DEFAULT_PEN_R = 0xb3
DEFAULT_PEN_G = 0xb8
DEFAULT_PEN_B = 0xff


class Turtle(object):
	""" The turtle class """

	_pose_pub = None
	_colour_pub = None

	# qreal lin_vel_;
	# qreal ang_vel_;
	_last_command_time = 0
	_x_vel = 0.0
	_y_vel = 0.0

	_pos = Point()

	# float meter_;

	def __init__(self, name, point):
		""" Ctor """

		object.__init__(self)

		assert(isinstance(point, Point))
		self._pos = point

		rospy.init_node(name)

		rospy.Subscriber(name + "/cmd_vel", Twist, self._velocity_callback)
		self._pose_pub = rospy.Publisher(name + "/pose", Pose, queue_size=10)
		self._colour_pub = rospy.Publisher(name + "/color_sensor", Color, queue_size=10)


	def _velocity_callback(self, data):
		""" Set the velocity based on the callback """

		self._last_command_time = rospy.Time.now()
		self._x_vel = data.linear.x
		self._y_vel = data.linear.y


	def update(self, dtime, background, canvas_width, canvas_height):
		"""
		Update the turtle state and position
		canvas_width: Expected to be max index of the x side
		canvas_height: Like canvas_width, but for y
		"""

		# Movement commands are only valid for one second
		if (rospy.Time.now() - self._last_command_time > rospy.Duration(1.0)):
			self._x_vel = 0.0
			self._y_vel = 0.0

		self._pos.x += self._x_vel * dtime
		self._pos.y += self._y_vel * dtime

		# Clamp to screen size
		if (self._pos.x < 0 or self._pos.x > canvas_width
			or self._pos.y < 0 or self._pos.y > canvas_height):
			rospy.logwarn("Oh no! I hit the wall! (Clamping from [x=%f, y=%f])", self._pos.x, self._pos.y)


		self._pos.x = min(max(float(self._pos.x), 0.0), float(canvas_width))
		self._pos.y = min(max(float(self._pos.y), 0.0), float(canvas_height))

		# Publish pose of the turtle
		pose = Pose()
		pose.x = self._pos.x
		pose.y = canvas_height - self._pos.y
		self._pose_pub.publish(pose)

		# Figure out (and publish) the color underneath the turtle
		colour = Color()
		rgb = background[self._pos.x][self._pos.y]
		colour.r = rgb.r
		colour.g = rgb.g
		colour.b = rgb.b
		self._colour_pub.publish(colour)

		rospy.logdebug("[%s]: pos_x: %f pos_y: %f", rospy.get_namespace(), self._pos.x, self._pos.y)


if __name__ == "__main__":
	T = Turtle("turtle1", Point())
	rospy.spin()
