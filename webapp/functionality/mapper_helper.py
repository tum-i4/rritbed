#!/usr/bin/env python
""" Mapper helper functions """

import collections
import math


class MapperBase(object):
	""" Base class for mappers """

	_X = "X"
	_Y = "Y"

	_SUB_SPACE_SIZE = {
		_X: 5,
		_Y: 5
	}


	def map(self, matrix, crd_x, crd_y):
		""" Maps given coordinates to the given matrix """

		# Throws if incorrect size
		self.assert_size(matrix)

		# Both throw if invalid value
		mapped_x = self.map_coordinate(crd_x, self._X)
		mapped_y = self.map_coordinate(crd_y, self._Y)

		return matrix[mapped_x][mapped_y]


	def map_coordinate(self, original_coordinate, dimension):
		""" Maps the original coordinate to our space """
		converted_coordinate = math.floor(original_coordinate / 500)

		if self._dimension_invalid(converted_coordinate, dimension):
			raise ArithmeticError("Resulting coordinate is invalid - "+
				"was the given value from an invalid space?\n" +
				"Given value: {}".format(original_coordinate))

		return converted_coordinate


	def assert_size(self, matrix):
		""" Verifies the given matrix' size """

		# pylint: disable-msg=C1801; (Do not use len in conditions - more understandable this way)
		if ((not isinstance(matrix, collections.Sequence))
			or len(matrix) == 0
			or (not isinstance(matrix[0], collections.Sequence))
			or len(matrix[0]) == 0):
			raise AssertionError("Given object is either no matrix or is zero in one or more dimensions")

		# We definitely have a two-dimensional matrix at this point

		if self._len_invalid(matrix, self._X) or self._len_invalid(matrix[0], self._Y):
			raise AssertionError("Given matrix has incorrect dimensions.\nx: {}\ny: {}".format(
				len(matrix), len(matrix[0])))


	def _dimension_invalid(self, value, dimension):
		return value < 0 or value >= self._SUB_SPACE_SIZE[dimension]


	def _len_invalid(self, seq, dimension):
		return len(seq) != self._SUB_SPACE_SIZE[dimension]
