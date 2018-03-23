#!/usr/bin/env python
""" Printer class """

from __future__ import print_function
import sys


class Printer(object):
	""" Printer that allows squelching output. """

	def __init__(self, squelch=False, name=None):
		""" Ctor """
		object.__init__(self)
		self.squelch = squelch
		self.name = name
		self.last_ended_in_newline = True


	def prt(self, message, preface=True, newline=True):
		""" Print if not squelching. Handles newline characters in the message. """

		for line in message.split("\n"):
			self._prt_line(line, preface, newline)


	def _prt_line(self, line, preface=True, newline=True):
		""" Print single line. """

		if self.squelch:
			return

		if preface and self.name and self.last_ended_in_newline:
			line = "[{}] {}".format(self.name, line)
		self.last_ended_in_newline = newline

		print(line, end="\n" if newline else "")
		sys.stdout.flush()
