#!/usr/bin/env python
""" Live IDS """

import os
import time
import uuid
from enum import Enum

class LiveIds(object):
	""" Live intrusion detection """

	LOG_DIR = "log"
	LOG_FILE_PREFIX = "log"

	def __init__(self):
		""" Ctor """

		object.__init__(self)


	def process(self, log_entry):
		""" Process the given entry. Outputs a warning when the detection was successful. """

		raise NotImplementedError()


# pylint: disable-msg=R0903; (Too few public methods)
class IdsClassification(Enum):
	""" IDS classification results """

	normal = 0
	intrusion = 1
