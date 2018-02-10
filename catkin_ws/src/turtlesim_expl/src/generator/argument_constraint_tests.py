#!/usr/bin/env python
""" Unit and system tests for LFO and related classes """

import unittest

from parameterized import parameterized

from argument_constraint import ArgumentConstraint


class Tests(unittest.TestCase):
	""" Tests for the generator module """

	@parameterized([
		(1, 1, 1), # all overlapping
		(-1, -1, 1), # default overlapping min
		(1, -1, 1), # default overlapping max
		(0, -2, 2) # default not overlapping
	])
	def test_ctor_valid(self, default_value, min_value, max_value):
		""" Tests for the constructor with valid inputs """
		# Test will fail if the constructor raises an error
		ArgumentConstraint(default_value, min_value, max_value)


	@parameterized([
		(1, 0, 0), # default > max
		(-1, 0, 0), # default < min
		(0, 1, -1), # max < min
	])
	def test_ctor_invalid(self, default_value, min_value, max_value):
		""" Tests for the constructor with invalid inputs """
		self.assertRaises(ArgumentConstraint(default_value, min_value, max_value))


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


	@parameterized([
		(0, 0, 0, 1), # too big
		(0, 0, 0, -1), # too small
		(0, -1, 1, 100), # way too big
		(0, -1, 1, -100) # way too small
	])
	def test_fits_doesnt(self, default_value, min_value, max_value, test_value):
		""" Tests invalid fits """
		self._test_fits(default_value, min_value, max_value, test_value, False)


	def _test_fits(self, default_value, min_value, max_value, test_value, expected):
		constraint = ArgumentConstraint(default_value, min_value, max_value)
		self.assertEqual(constraint.fits(test_value), expected)
