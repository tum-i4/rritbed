#!/usr/bin/env python
""" POI mapper """

from functionality.mapper_base import MapperBase

class PoiMapper(MapperBase):
	""" Mapping coordinates to one of the POIs """

	ita = "Italian"
	ger = "German"
	frc = "French"

	restaurants = [
		[ita, ita, ger, ger, frc],
		[ger, ita, ger, ger, ger],
		[ger, ita, frc, ger, frc],
		[ita, ger, ger, ger, frc],
		[ita, ita, frc, ger, frc]
	]

	tot = "Total"
	shl = "Shell"
	arl = "Aral"

	gas_stations = [
		[tot, tot, arl, arl, tot],
		[arl, tot, arl, arl, arl],
		[shl, tot, shl, shl, shl],
		[tot, shl, shl, shl, shl],
		[tot, arl, tot, tot, arl]
	]

	restaurants_field = "restaurant"
	gas_stations_field = "gas station"

	pois = {
		restaurants_field: restaurants,
		gas_stations_field: gas_stations
	}

	@staticmethod
	def map(poi_type, crd_x, crd_y):
		"""
		Map the given coordinates to a POI of the given type.
		returns: "Invalid" for invalid types.
		"""

		if poi_type not in PoiMapper.pois:
			return "Invalid"

		return MapperBase._map(PoiMapper.pois[poi_type], crd_x, crd_y)
