#!/usr/bin/env python
""" Module for the PoseProcessor class """

class PoseProcessor(object):
	""" Process poses and convert them to request objects. """

	_name = ""
	_processor = None

	def __init__(self, name, processor):
		""" Ctor """

		object.__init__(self)

		self._name = name
		self._processor = processor


	def process(self, crd_x, crd_y, request):
		""" Process the given coordinates and enrich the request with result. """
		pass
