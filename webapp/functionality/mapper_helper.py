#!/usr/bin/env python
""" Mapper helper functions """

import collections
import math


_SUB_SPACE_SIZE = 5


def map_coordinate(original_coordinate):
	""" Maps the original coordinate to our space """
	converted_coordinate = math.floor(original_coordinate / 500)

	if _dimension_invalid(converted_coordinate):
		raise ArithmeticError("Resulting coordinate is invalid - "+
			"was the given value from an invalid space?\n" +
			"Given value: {}".format(original_coordinate))

	return converted_coordinate


def assert_size(matrix):
	""" Verifies the given matrix' size """

	# pylint: disable-msg=C1801; (Do not use len in conditions - more understandable this way)
	if ((not isinstance(matrix, collections.Sequence))
		or len(matrix) == 0
		or (not isinstance(matrix[0], collections.Sequence))
		or len(matrix[0]) == 0):
		raise AssertionError("Given object is either no matrix or is zero in one or more dimensions")

	# We definitely have a two-dimensional matrix at this point

	if _len_invalid(matrix) or _len_invalid(matrix[0]):
		raise AssertionError("Given matrix has incorrect dimensions.\nx: {}\ny: {}".format(
			len(matrix), len(matrix[0])))


def _dimension_invalid(value):
	return value < 0 or value >= _SUB_SPACE_SIZE

def _len_invalid(seq):
	return len(seq) != _SUB_SPACE_SIZE
