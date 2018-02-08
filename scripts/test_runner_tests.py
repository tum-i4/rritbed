#!/usr/bin/env python
""" Unit tests for the test runner """

# pylint: disable-msg=W0212; (Illegal access to private members)

import os
import unittest

from test_runner import TestRunner


class Tests(unittest.TestCase):
	""" Tests for the TestRunner """

	test_path = os.path.expanduser("~/test/test_runner_test/")
	valid_module = "file_tests.py"
	invalid_module = "bla/bla_tests.py"

	def test_discover_valid(self):
		""" Test discovery on valid pattern """

		expected = [self.test_path + x for x in [self.valid_module, self.invalid_module]]
		result = TestRunner._discover(self.test_path, TestRunner.DEFAULT_PATTERN)

		self.assertListEqual(expected, result)


	def test_discover_invalid(self):
		""" Test discovery on invalid pattern """

		expected = []
		result = TestRunner._discover(self.test_path, "cant-find-anything")

		self.assertListEqual(expected, result)


	def test_load_module_valid(self):
		""" Tests loading a valid test module """

		pass


if __name__ == "__main__":
	raise NotImplementedError("Class was built to be run by a TestRunner")
