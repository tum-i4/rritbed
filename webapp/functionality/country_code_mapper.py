#!/usr/bin/env python
""" Country code mapper """

import math

class CountryCodeMapper(object):
	""" Mapping coordinates to country codes """

	codes = [
		["DE", "DE", "DE", "AT", "AT"],
		["DE", "DE", "DE", "AT", "AT"],
		["DE", "DE", "DE", "CH", "AT"],
		["FR", "DE", "DE", "CH", "AT"],
		["FR", "FR", "DE", "IT", "IT"],
	]

	@staticmethod
	def map(crd_x, crd_y):
		"""
		Maps given coordinates to country code
		returns: "DE" if coordinates are malformed
		"""

		reduced_rounded_x = math.floor(crd_x / 500)
		reduced_rounded_y = math.floor(crd_y / 500)

		# Coordinate is malformed: Return default
		if (reduced_rounded_x < 0 or reduced_rounded_y < 0
			or reduced_rounded_x >= len(CountryCodeMapper.codes)
			or reduced_rounded_y >= len(CountryCodeMapper.codes[0])):
			return "DE"

		return CountryCodeMapper.codes[reduced_rounded_x][reduced_rounded_y]
