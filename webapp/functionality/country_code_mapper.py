#!/usr/bin/env python
""" Country code mapper """

from functionality.mapper_helper import map_coordinate, assert_size

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
		assert_size(CountryCodeMapper.codes)

		# Throw if invalid value
		mapped_x = map_coordinate(crd_x)
		mapped_y = map_coordinate(crd_y)

		return CountryCodeMapper.codes[mapped_x][mapped_y]
