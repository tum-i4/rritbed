#!/usr/bin/env python
"""
Rebuilding turtle.cpp in Python
"""

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
	
	# ros::ServiceServer set_pen_srv_;
	# ros::ServiceServer teleport_relative_srv_;
	# ros::ServiceServer teleport_absolute_srv_;

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


		# val_future_color_pub = nh_.advertise<Color>("future_color_sensor", 1);
		# set_pen_srv_ = nh_.advertiseService("set_pen", &Turtle::setPenCallback, this);
		# teleport_relative_srv_ = nh_.advertiseService("teleport_relative", &Turtle::teleportRelativeCallback, this);
		# teleport_absolute_srv_ = nh_.advertiseService("teleport_absolute", &Turtle::teleportAbsoluteCallback, this);

		# meter_ = turtle_image_.height();
		# rotateImage();
	

	def _velocityCallback(self, data):
		pass
