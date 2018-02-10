#!/usr/bin/env python
""" Unit and system tests for LFO and related classes """

import unittest

from parameterized import parameterized

from argument_constraint import ArgumentConstraint


class Tests(unittest.TestCase):
	""" Tests for the generator module """

	@parameterized([
		(1, 1, 1, 1), # hits both ends
		(0, -1, 1, 0), # is default value
		(0, -1, 1, -1), # hits min
		(0, -1, 1, 1), # hits max
		(0, -2, 2, 1) # random value
	])
	def test_fits_does(self, default_value, min_value, max_value, test_value):
		""" Tests valid fits """
		self._test_fits(default_value, min_value, max_value, test_value, True)


	def _test_fits(self, default_value, min_value, max_value, test_value, expected):
		constraint = ArgumentConstraint(default_value, min_value, max_value)
		self.assertEqual(constraint.fits(test_value), expected)
