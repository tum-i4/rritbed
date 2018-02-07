#!/usr/bin/env python
""" Mapper helper functions """

import collections
import math


class MapperBase(object):
	""" Base class for mappers """

	_X_FIELD = "X"
	_Y_FIELD = "Y"

	_SUB_SPACE_SIZE = {
		_X_FIELD: 5,
		_Y_FIELD: 5
	}


	@staticmethod
	def _map(matrix, crd_x, crd_y):
		""" Maps given coordinates to the given matrix """

		# Throws if incorrect size
		MapperBase._assert_size(matrix)

		# Both throw if invalid value
		crd_x = float(crd_x)
		crd_y = float(crd_y)

		# Both throw if invalid value
		mapped_x = MapperBase._map_coordinate(crd_x, MapperBase._X_FIELD)
		mapped_y = MapperBase._map_coordinate(crd_y, MapperBase._Y_FIELD)

		return matrix[mapped_x][mapped_y]


	@staticmethod
	def _map_coordinate(original_coordinate, dimension):
		""" Maps the original coordinate to our space """

		converted_coordinate = (
			int(
				math.floor(
					original_coordinate / 500.0)))

		if MapperBase._dimension_invalid(converted_coordinate, dimension):
			raise ArithmeticError("Resulting coordinate is invalid - "+
				"was the given value from an invalid space?\n" +
				"Given value: {}".format(original_coordinate))

		return converted_coordinate


	@staticmethod
	def _assert_size(matrix):
		""" Verifies the given matrix' size """

		# pylint: disable-msg=C1801; (Do not use len in conditions - more understandable this way)
		if ((not isinstance(matrix, collections.Sequence))
			or len(matrix) == 0
			or (not isinstance(matrix[0], collections.Sequence))
			or len(matrix[0]) == 0):
			raise AssertionError("Given object is either no matrix or is zero in one or more dimensions")

		# We definitely have a two-dimensional matrix at this point

		if (MapperBase._len_invalid(matrix, MapperBase._get_width())
			or MapperBase._len_invalid(matrix[0], MapperBase._get_height())):
			raise AssertionError("Given matrix has incorrect dimensions.\nx: {}\ny: {}".format(
				len(matrix), len(matrix[0])))


	@staticmethod
	def _dimension_invalid(value, dimension):
		return value < 0 or value >= MapperBase._SUB_SPACE_SIZE[dimension]


	@staticmethod
	def _len_invalid(seq, dimension):
		return len(seq) != MapperBase._SUB_SPACE_SIZE[dimension]


	@staticmethod
	def _get_width():
		return MapperBase._SUB_SPACE_SIZE[MapperBase._X_FIELD]


	@staticmethod
	def _get_height():
		return MapperBase._SUB_SPACE_SIZE[MapperBase._X_FIELD]
