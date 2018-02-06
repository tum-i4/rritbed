#!/usr/bin/env python
""" Mapper helper functions """

import math


def map_coordinate(original_coordinate):
	""" Maps the original coordinate to our space """
	converted_coordinate = math.floor(original_coordinate / 500)

	if converted_coordinate > 5 or converted_coordinate < 0:
		raise ArithmeticError("Resulting coordinate too big - "+
			"was the given value from a bigger space?\n" +
			"Given value: {}".format(original_coordinate))

	return converted_coordinate
