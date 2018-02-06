#!/usr/bin/env python
""" POI mapper """

from functionality.mapper_helper import map_coordinate, assert_size

class PoiMapper(object):
	""" Mapping coordinates to one of the POIs """

	ita = "ita"
	ger = "ger"
	frc = "frc"

	restaurants = [
		[ita, ita, ger, ger, frc],
		[ger, ita, ger, ger, ger],
		[ger, ita, frc, ger, frc],
		[ita, ger, ger, ger, frc],
		[ita, ita, frc, ger, frc]
	]

	@staticmethod
	def map(crd_x, crd_y, type):
		""" Maps the given coordinates and type to a POI """

		assert_size(PoiMapper.restaurants)

		pass
