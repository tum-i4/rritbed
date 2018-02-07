#!/usr/bin/env python
""" Unit tests for the test runner """

# pylint: disable-msg=W0212; (Illegal access to private members)

import os
import unittest

from test_runner import TestRunner


class Tests(unittest.TestCase):
	""" Tests for the TestRunner """

	test_path = os.path.expanduser("~/test/test_runner_test")

	def test_discover_valid(self):
		""" Test discovery on valid pattern """

		expected = [self.test_path + x for x in ["/file_tests.py", "/bla/bla_tests.py"]]
		result = TestRunner._discover(self.test_path, TestRunner.DEFAULT_PATTERN)

		self.assertListEqual(expected, result)


if __name__ == "__main__":
	raise NotImplementedError("Class was built to be run by a TestRunner")
