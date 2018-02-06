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

	tot = "tot"
	shl = "shl"
	arl = "arl"

	gas_stations = [
		[tot, tot, arl, arl, tot],
		[arl, tot, arl, arl, arl],
		[shl, tot, shl, shl, shl],
		[tot, shl, shl, shl, shl],
		[tot, arl, tot, tot, arl]
	]

	@staticmethod
	def map(crd_x, crd_y, type):
		""" Maps the given coordinates and type to a POI """

		assert_size(PoiMapper.restaurants)
		assert_size(PoiMapper.gas_stations)

		pass
