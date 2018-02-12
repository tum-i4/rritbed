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

DEFAULT_PEN_R = 0xb3
DEFAULT_PEN_G = 0xb8
DEFAULT_PEN_B = 0xff

PI = 3.14159265


class Turtle(object):
	""" The turtle class """

	_pose_pub
	_colour_pub

	# QImage turtle_image_;
	# QImage turtle_rotated_image_;

	# QPointF pos_;
	# qreal orient_;

	# qreal lin_vel_;
	# qreal ang_vel_;
	# bool pen_on_;
	# QPen pen_;

	# ros::WallTime last_command_time_;

	# float meter_;

	# const QImage& turtle_image, const QPointF& pos, float orient
	def __init__(self):
		""" Ctor """

		object.__init__(self)

		rospy.init_node("turtle", anonymous=True)

		rospy.Subscriber("cmd_vel", Twist, self._velocityCallback)
		self._pose_pub = rospy.Publisher("pose", Pose)
		self._colour_pub = rospy.Publisher("color_sensor", Color)

		# meter_ = turtle_image_.height();
		# rotateImage();


	def _velocityCallback(self, data):
		pass
