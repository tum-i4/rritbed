#!/usr/bin/env python
""" IdsEntry class """

import attr


@attr.s
class IdsEntry(object):
	app_id = attr.ib()
	vector = attr.ib()
	my_class = attr.ib()
