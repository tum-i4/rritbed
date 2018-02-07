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
		MapperBase._assert_matrix_validity(matrix)

		# Both throw if invalid value
		crd_x = float(crd_x)
		crd_y = float(crd_y)

		# Both throw if invalid value
		mapped_x = MapperBase._map_coordinate(crd_x, MapperBase._get_width())
		mapped_y = MapperBase._map_coordinate(crd_y, MapperBase._get_height())

		return matrix[mapped_x][mapped_y]


	@staticmethod
	def _map_coordinate(original_coordinate, size_bound):
		""" Maps the original coordinate to our space """

		converted_coordinate = (
			int(
				math.floor(
					(size_bound - 1) * (
						original_coordinate / 500.0))))

		if MapperBase._dimension_invalid(converted_coordinate, size_bound):
			raise ArithmeticError("Resulting coordinate is invalid - "+
				"was the given value from an invalid space?\n" +
				"Given value: {}".format(original_coordinate))

		return converted_coordinate


	@staticmethod
	def _assert_matrix_validity(matrix):
		""" Verifies the given matrix' size """

		# pylint: disable-msg=C1801; (Do not use len in conditions - more understandable this way)
		if ((not isinstance(matrix, collections.Sequence))
			or len(matrix) == 0
			or (not isinstance(matrix[0], collections.Sequence))
			or len(matrix[0]) == 0):
			raise AssertionError("Given object is either no matrix or is zero in one or more dimensions")

		# We definitely have a two-dimensional matrix at this point

		if MapperBase._matrix_size_invalid(matrix):
			raise AssertionError("Given matrix has incorrect dimensions.\nx: {}\ny: {}".format(
				len(matrix), len(matrix[0])))


	@staticmethod
	def _dimension_invalid(value, size_bound):
		return value < 0 or value >= size_bound


	@staticmethod
	def _matrix_size_invalid(matrix):
		return (
			len(matrix) != MapperBase._get_width()
			and len(matrix[0]) != MapperBase._get_height())


	@staticmethod
	def _get_width():
		return MapperBase._SUB_SPACE_SIZE[MapperBase._X_FIELD]


	@staticmethod
	def _get_height():
		return MapperBase._SUB_SPACE_SIZE[MapperBase._X_FIELD]
