#!/usr/bin/env python
"""
Rebuilding turtle_frame.cpp in Python
"""

# Currently not supported:
# - graphical output
#   - 2D space
#   - turtle images
#   - all Qt fields and callbacks
# - services
#   - clear
#   - reset
#   - spawn
#   - kill
# - methods
#   - clear

# pylint: disable-msg=R0903; (Too few public methods)


import os
import random
from turtle import Turtle

import rospy
from util.rgb import Rgb
from util.point import Point

DEFAULT_BG_R = 0x45
DEFAULT_BG_G = 0x56
DEFAULT_BG_B = 0xff


class TurtleFrame(object):
	""" The frame for all turtles """

	_background = [[]]
	_turtles = {}

	_id_counter = 0
	_update_interval = None

	_has_gui = False
	_gui_size = 0
	_frame_count = 0
	_gui_output = None


	def __init__(self, draw_gui=False, intrusion=None):
		""" Ctor """

		object.__init__(self)

		self._has_gui = draw_gui

		# Initialise background (500 x 500)
		self._background = [[
			Rgb(DEFAULT_BG_R, DEFAULT_BG_G, DEFAULT_BG_B) for _ in range(0, 500)
			] for _ in range(0, 500)]

		rospy.set_param("background_r", DEFAULT_BG_R)
		rospy.set_param("background_g", DEFAULT_BG_G)
		rospy.set_param("background_b", DEFAULT_BG_B)

		rospy.loginfo("Starting turtle frame with parent node %s", rospy.get_name())

		trt_x = random.randrange(0, self.get_width())
		trt_y = random.randrange(0, self.get_height())

		self._spawn_turtle(trt_x, trt_y)

		# Colouring the background
		# Window is 500 x 500, starting bottom left at 0,0 and ending top right at 499,499

		# Top left: Pastel purple
		self._draw_area(Rgb.pastel_purple(), Point(0, 250), Point(249, 499))
		# Top right: Pastel yellow
		self._draw_area(Rgb.pastel_yellow(), Point(250, 250), Point(499, 499))
		# Bottom left: Pastel green
		self._draw_area(Rgb.pastel_green(), Point(0, 0), Point(249, 249))
		# Bottom right: Pastel blue
		self._draw_area(Rgb.pastel_blue(), Point(250, 0), Point(499, 249))
		# Intrusion zone (middle): Red
		self._draw_red(intrusion)

		# Initialise GUI (if requested)
		self._redraw()

		# Initialise update timer (16 msec)
		self._update_interval = rospy.Duration(0.016)
		rospy.Timer(self._update_interval, self._update_turtles)


	def get_width(self):
		""" Get the current background width. """
		return len(self._background) if self._background else 0


	def get_height(self):
		""" Get the current background height. """
		return len(self._background[0]) if self.get_width() > 0 and self._background[0] else 0


	def _draw_area(self, colour, from_point, to_point):
		"""
		Draw defined area in defined colour.\n
		colour: Instance of Rgb class\n
		top_left/bottom_right: Instances of Point class
		"""

		assert(isinstance(colour, Rgb))
		assert(isinstance(from_point, Point))
		assert(isinstance(to_point, Point))

		# We make sure that from < to as range can only go from small to big.
		from_x = min(from_point.x, to_point.x)
		to_x = max(from_point.x, to_point.x)
		from_y = min(from_point.y, to_point.y)
		to_y = max(from_point.y, to_point.y)

		# BL x, TR y      === TR ===
		# === BL ===      TR x, BL y

		# pylint: disable-msg=C0103; (Invalid variable names x, y)
		for x in range(from_x, to_x + 1):
			for y in range(from_y, to_y + 1):
				self._background[x][y] = colour


	def _draw_red(self, intrusion):
		pass


	def _spawn_turtle(self, trt_x, trt_y, name=None):
		""" Add a turtle to the field at the given coordinates. """

		if name is None or name == "":
			name = self._create_unique_turtle_name()
		elif self._has_turtle(name):
			return ""

		turtle = Turtle(name, Point(trt_x, trt_y))
		self._turtles[name] = turtle

		rospy.loginfo("New turtle [%s] at x=[%d], y=[%d]", name, trt_x, trt_y)

		return name


	def _create_unique_turtle_name(self):
		""" Increase the ID counter until a unique name of the form "turtle<id>" is found. """

		self._id_counter += 1
		new_name = "turtle{}".format(self._id_counter)

		if self._has_turtle(new_name):
			return self._create_unique_turtle_name()

		return new_name


	def _has_turtle(self, name):
		""" Check the turtles for the given name. """
		return name in self._turtles


	def _update_turtles(self, _):
		""" Update callback: Call update() on all turtles and redraws GUI. """

		modified = False
		for key in self._turtles:
			modified |= self._turtles[key].update(
				self._update_interval.to_sec(), self._background, self.get_width(), self.get_height())

		if modified:
			self._redraw()

		self._frame_count += 1


	def _redraw(self):
		""" Draw an updated GUI output. """

		if not self._has_gui:
			return

		rows, _ = os.popen('stty size', 'r').read().split()
		self._gui_size = int(rows)

		if self._gui_output is None or len(self._gui_output) != self._gui_size:
			self._gui_output = [["" for _ in range(0, self._gui_size)] for _ in range(0, self._gui_size)]

		self._update_output()

		os.system("clear")

		# pylint: disable-msg=C0103; (Invalid variable names x, y)
		# We draw from top left (0,49) to bottom right (49,0)
		for y in range(self._gui_size - 1, -1, -1):
			line_output = ""
			for x in range(0, self._gui_size):
				line_output += self._gui_output[x][y] + " "
			print(line_output)


	def _update_output(self):
		""" Update the GUI output in scale of self._gui_size. Turtle is marked with index. """

		scale = min(self.get_height(), float(self.get_height()) / self._gui_size)

		# pylint: disable-msg=C0103; (Invalid variable names x, y)
		for x in range(0, self._gui_size):
			for y in range(0, self._gui_size):
				self._gui_output[x][y] = self._get_output_letter(
					self._background[int(x * scale)][int(y * scale)])

		for name, turtle in self._turtles.items():
			trt_x = int(turtle.pos.x / scale)
			trt_y = int(turtle.pos.y / scale)
			for x in range(max(trt_x-1, 0), min(trt_x+2, self._gui_size)):
				for y in range(max(trt_y-1, 0), min(trt_y+2, self._gui_size)):
					self._gui_output[x][y] = " "

			self._gui_output[trt_x][trt_y] = str(self._turtles.keys().index(name) + 1)


	@staticmethod
	def _get_output_letter(rgb):
		""" Produce a letter for the given Rgb. Returns "?" for unknown colours. """

		if rgb == Rgb.pastel_purple():
			return "p"
		elif rgb == Rgb.pastel_yellow():
			return "y"
		elif rgb == Rgb.pastel_green():
			return "g"
		elif rgb == Rgb.pastel_blue():
			return "b"
		elif rgb == Rgb.red():
			return " "

		return "?"
