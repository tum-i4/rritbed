#!/usr/bin/env python
""" Country code mapper """

from functionality.mapper_base import MapperBase

class CountryCodeMapper(MapperBase):
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

		return MapperBase._map(CountryCodeMapper.codes, crd_x, crd_y)
