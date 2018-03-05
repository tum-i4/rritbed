#!/usr/bin/env python
""" IdsClassification enum """

from enum import Enum


# pylint: disable-msg=R0903; (Too few public methods)
class IdsClassification(Enum):
	""" IDS classification results """

	normal = 0
	intrusion = 1
