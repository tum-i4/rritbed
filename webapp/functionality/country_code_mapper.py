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
		"""
		Maps given coordinates to country code
		returns: "DE" if coordinates are malformed
		"""

		# Throws if incorrect size
		assert_size(CountryCodeMapper.codes)

		mapped_x = map_coordinate(crd_x)
		mapped_y = map_coordinate(crd_y)

		# Coordinate is malformed: Return default
		if (mapped_x < 0 or mapped_y < 0
			or mapped_x >= len(CountryCodeMapper.codes)
			or mapped_y >= len(CountryCodeMapper.codes[0])):
			return "DE"

		return CountryCodeMapper.codes[mapped_x][mapped_y]
