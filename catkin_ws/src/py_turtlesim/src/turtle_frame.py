#!/usr/bin/env python
"""
Rebuilding turtle_frame.cpp in Python
"""

# Currently not supported:
# - graphical output
#   - 2D space
#   - turtle images
# - services
#   - clear
#   - reset
#   - spawn
#   - kill
# - methods
#   - hasTurtle

import random
import rospy

from turtle import Turtle
from util.rgb import Rgb
from util.point import Point

DEFAULT_BG_R = 0x45
DEFAULT_BG_G = 0x56
DEFAULT_BG_B = 0xff


class TurtleFrame(object):
	""" The turtle frame class """

	_width = 0
	_height = 0
	_2d_plane = [[]]
	_turtles = {}
	_id_counter = 0

	# path_image_(500, 500, QImage::Format_ARGB32)
	# path_painter_(&path_image_)
	# frame_count_(0)
	# id_counter_(0)

	# ros::NodeHandle nh_;
	# QTimer* update_timer_;
	# QImage path_image_;
	# QPainter path_painter_;

	# uint64_t frame_count_;

	# ros::WallTime last_turtle_update_;

	# typedef std::map<std::string, TurtlePtr> M_Turtle;

	# float meter_;
	# float width_in_meters_;
	# float height_in_meters_;

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		# Initialise plain (500 x 500)
		self._2d_plane = [[Rgb(DEFAULT_BG_R, DEFAULT_BG_G, DEFAULT_BG_B)] * 500] * 500
		self._width = len(self._2d_plane)
		self._height = len(self._2d_plane[0])

		# Initialise update timer (16 msec)

		#   srand(time(NULL));

		#   update_timer_ = new QTimer(this);
		#   update_timer_->setInterval(16);
		#   update_timer_->start();

		#   connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

		rospy.init_node("turtle_frame")
		rospy.set_param("background_r", DEFAULT_BG_R)
		rospy.set_param("background_g", DEFAULT_BG_G)
		rospy.set_param("background_b", DEFAULT_BG_B)

		#   clear();

		rospy.loginfo("Starting turtle frame, %s", rospy.get_name())

		trt_x = random.randrange(0, self._width)
		trt_y = random.randrange(0, self._height)

		self._spawn_turtle(trt_x, trt_y)

		#   // === VAU ===
		#   // Window is 500 x 500, starting top left at 0,0 and ending bottom right at 500,500

		#   // TOP LEFT: PURPLE
		#   QColor pastelPurple = QColor(150, 125, 210);
		#   vauDrawArea(pastelPurple, QPoint(0, 0), QPoint(250, 250));

		#   // // TOP RIGHT: YELLOW
		#   QColor pastelYellow = QColor(255, 240, 120);
		#   vauDrawArea(pastelYellow, QPoint(250, 0), QPoint(500, 250));

		#   // // BOTTOM LEFT: GREEN
		#   QColor pastelGreen = QColor(100, 180, 100);
		#   vauDrawArea(pastelGreen, QPoint(0, 250), QPoint(250, 500));

		#   // // BOTTOM RIGHT: BLUE
		#   QColor pastelBlue = QColor(100, 180, 250);
		#   vauDrawArea(pastelBlue, QPoint(250, 250), QPoint(500, 500));

		#   // DANGER ZONE MIDDLE: RED
		#   vauDrawArea(Qt::red, QPoint(245, 245), QPoint(255, 255));

		#   // END TODO

		#   // spawn all available turtle types
		#   if(false)
		#   {
		#     for(int index = 0; index < turtles.size(); ++index)
		#     {
		#       QString name = turtles[index];
		#       name = name.split(".").first();
		#       name.replace(QString("-"), QString(""));
		#       spawnTurtle(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
		#     }
		#   }


	def _draw_area(self, colour, top_left, bottom_right):
		""" Draw defined area in defined colour """

		assert(isinstance(colour, Rgb))
		assert(isinstance(top_left, Point))
		assert(isinstance(bottom_right, Point))

		# === TL ===      BR x, TL y
		# TL x, BR y      === BR ===

		# pylint: disable-msg=C0103; (Invalid variable names x, y)
		for x in range(top_left.x, bottom_right.x + 1):
			for y in range(top_left.y, bottom_right.y + 1):
				self._2d_plane[x][y] = colour


	def _spawn_turtle(self, trt_x, trt_y, name=None):
		""" Add a turtle to the field at the given coordinates """

		if name is None or name == "":
			name = self._create_unique_turtle_name()
		elif self._has_turtle(name):
			return ""

		turtle = Turtle(name, Point(trt_x, trt_y))
		self._turtles[name] = turtle

		rospy.loginfo("New turtle at [%s] at x=[%f], y=[%f]", name, trt_x, trt_y)

		return name


	def _create_unique_turtle_name(self):
		""" Increases the ID counter until a unique name of the form "turtle<id>" is found """

		new_name = "turtle" + self._id_counter
		self._id_counter += 1

		if self._has_turtle(new_name):
			return self._create_unique_turtle_name()

		return new_name


	def _has_turtle(self, name):
		""" Checks the turtles for the given name """
		return name in self._turtles
