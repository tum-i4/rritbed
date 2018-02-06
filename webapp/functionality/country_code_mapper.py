#!/usr/bin/env python
""" Country code mapper """

from functionality.mapper_base import MapperBase

class CountryCodeMapper(object):
	""" Mapping coordinates to country codes """

	codes = [
		["DE", "DE", "DE", "AT", "AT"],
		["DE", "DE", "DE", "AT", "AT"],
		["DE", "DE", "DE", "CH", "AT"],
		["FR", "DE", "DE", "CH", "AT"],
		["FR", "FR", "DE", "IT", "IT"]
	]

	@staticmethod
	def map(crd_x, crd_y):
		""" Maps given coordinates to country code """

		# Throws if incorrect size
		MapperBase.assert_size(CountryCodeMapper.codes)

		# Throw if invalid value
		mapped_x = MapperBase.map_coordinate(crd_x, MapperBase._X)
		mapped_y = MapperBase.map_coordinate(crd_y, MapperBase._Y)

		return CountryCodeMapper.codes[mapped_x][mapped_y]
