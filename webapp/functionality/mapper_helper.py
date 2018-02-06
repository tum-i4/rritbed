#!/usr/bin/env python
""" Mapper helper functions """

import collections
import math


def map_coordinate(original_coordinate):
	""" Maps the original coordinate to our space """
	converted_coordinate = math.floor(original_coordinate / 500)

	if converted_coordinate > 5 or converted_coordinate < 0:
		raise ArithmeticError("Resulting coordinate too big - "+
			"was the given value from a bigger space?\n" +
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

	if len(matrix) != 5 or len(matrix[0]) != 5:
		raise AssertionError("Given matrix has incorrect dimensions.\nx: {}\ny: {}".format(
			len(matrix), len(matrix[0])))
