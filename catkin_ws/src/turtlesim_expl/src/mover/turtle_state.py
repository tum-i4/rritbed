#!/usr/bin/env python
""" TurtleState class """

import time
import attr


@attr.s
class TurtleState(object):
	""" Hold a data object and last broadcast time in seconds. """

	data = attr.ib()
	last_update = attr.ib(default=0)


	def update(self, data):
		""" Update the data and set the last_update field to now. """

		self.data = data
		self.last_update = time.time()


	def get_time_since(self):
		""" Get the time that has passed since the last update. """

		time_now = time.time()
		return time_now - self.last_update
