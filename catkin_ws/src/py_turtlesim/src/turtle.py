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

from util.pos import Pos

DEFAULT_PEN_R = 0xb3
DEFAULT_PEN_G = 0xb8
DEFAULT_PEN_B = 0xff

PI = 3.14159265

NS = "turtle1"


class Turtle(object):
	""" The turtle class """

	_pose_pub = None
	_colour_pub = None

	# qreal lin_vel_;
	# qreal ang_vel_;
	_last_command_time = 0
	_x_vel = 0.0
	_y_vel = 0.0

	_pos = Pos()

	# QImage turtle_image_;
	# QImage turtle_rotated_image_;

	# QPointF pos_;
	# qreal orient_;

	# bool pen_on_;
	# QPen pen_;

	# ros::WallTime last_command_time_;

	# float meter_;

	# const QImage& turtle_image, float orient
	def __init__(self, pos):
		""" Ctor """

		object.__init__(self)

		assert(isinstance(pos, Pos))
		self._pos = pos

		rospy.init_node("turtle", anonymous=True)

		rospy.Subscriber(NS + "/cmd_vel", Twist, self._velocity_callback)
		self._pose_pub = rospy.Publisher(NS + "/pose", Pose, queue_size=10)
		self._colour_pub = rospy.Publisher(NS + "/color_sensor", Color, queue_size=10)

		# meter_ = turtle_image_.height();
		# rotateImage();


	def _velocity_callback(self, data):
		""" Set the velocity based on the callback """

		self._last_command_time = rospy.Time.now()
		self._x_vel = data.linear.x
		self._y_vel = data.linear.y


	def update(self, path_image, canvas_width, canvas_height):
		"""
		Update the turtle state and position
		canvas_width: Expected to be max index of the x side
		canvas_height: Like canvas_width, but for y
		"""

		# Movement commands are only valid for one second
		if (rospy.Time.now() - self._last_command_time > rospy.Duration(1.0)):
			self._x_vel = 0.0
			self._y_vel = 0.0

		self._pos.x += self._x_vel
		self._pos.y += self._y_vel

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

		pass


if __name__ == "__main__":
	T = Turtle(Pos())
	rospy.spin()
