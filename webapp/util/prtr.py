#!/usr/bin/env python
""" Printer class """

from __future__ import print_function
import sys
import time


class Printer(object):
	""" Printer that allows squelching output. """

	def __init__(self, squelch=False, verbose=True, name=None):
		""" Ctor """

		object.__init__(self)

		self.squelch = squelch
		self.verbose = verbose
		self.name = name
		self.last_ended_in_newline = True


	def prt(self, message, only_verbose=False, preface=True, newline=True):
		"""
		Print if not squelching. Handles newline characters in the message.
		: param only_verbose : Pass True to only print the message in verbose mode.
		"""

		for line in message.split("\n"):
			self._prt_line(line, only_verbose, preface, newline)


	def _prt_line(self, line, only_verbose, preface, newline):
		""" Print single line. """

		if self.squelch or only_verbose and not self.verbose:
			return

		if preface and self.name and self.last_ended_in_newline:
			line = self._preface_line(line)
		self.last_ended_in_newline = newline

		print(line, end="\n" if newline else "")
		sys.stdout.flush()


	def _preface_line(self, line):
		""" Preface the given line with the contained name. """
		return "[{}] {}".format(self.name, line)



class Storer(Printer):
	"""
	Printer replacement that stores all lines given to it.
	Does NOT support verbosity or non-newlines when storing!
	"""

	def __init__(self, squelch=False, verbose=True, name=None):
		""" Ctor """

		super(Storer, self).__init__(squelch, verbose, name)
		self.lines = []


	def prt(self, message, only_verbose=False, preface=True, newline=True):
		""" Save the given message. """

		self.lines.append((message, only_verbose, preface, newline))


	def get_messages(self, purge=False):
		""" Retrieve the stored messages. Optionally purge them. """

		lines = self.lines
		if purge:
			self.lines = []

		messages = []
		for message, _, _, _ in lines:
			messages.append(message)

		return messages


	def printout(self):
		""" Print out the stored lines. Empties the contained lines. """

		for message, only_verbose, preface, newline in self.lines:
			super(Storer, self)._prt_line(message, only_verbose, preface, newline)

		self.lines = []



class TimePrinter(Printer):
	"""
	Printer replacement that additionally prefixes each new line with the current time.
	"""

	def __init__(self, squelch=False, verbose=True, name=None):
		""" Ctor """

		super(TimePrinter, self).__init__(squelch, verbose, "INVALID")
		self.printer_name = name


	def prt(self, message, only_verbose=False, preface=True, newline=True):
		""" Print if not squelching, adding current time.
		See Printer.prt() for implementation. """

		if preface:
			time_str = time.strftime("%H:%M:%S")
			message = "{} - {}".format(time_str, message)

		super(TimePrinter, self).prt(message, only_verbose, preface, newline)
