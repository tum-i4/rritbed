#!/usr/bin/env python
""" IdsResult class and Classification enum """

# pylint: disable-msg=R0903; (Too few public methods)

from enum import Enum
import attr


class Classification(Enum):
	""" IDS classification enum """

	normal = 0
	intrusion = 1


@attr.s
class IdsResult(object):
	""" Classification and confidence value """
	classification = attr.ib(validator=attr.validators.instance_of(Classification))
	confidence = attr.ib(validator=attr.validators.in_(range(0, 101)))
