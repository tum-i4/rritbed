#!/usr/bin/env python
""" Unit tests for the test runner """

# pylint: disable-msg=W0212; (Illegal access to private members)

import os
import unittest

from test_runner import TestRunner


class Tests(unittest.TestCase):
	""" Tests for the TestRunner """

	test_path = os.path.expanduser("~/test/test_runner_test")


if __name__ == "__main__":
	raise NotImplementedError("Class was built to be run by a TestRunner")
