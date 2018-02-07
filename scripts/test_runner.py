#!/usr/bin/env python
""" Unit and system tests for LFO and related classes """

import argparse
import imp
import os
import re
import unittest


class TestRunner(object):
	"""
	Discovers tests in given directory and all subdirectories.
	Runs all found test suites and prints results for each.
	"""

	@staticmethod
	def run_with_args():
		""" Parse arguments and run accordingly """

		parser = argparse.ArgumentParser(prog="tr",
			description="Discover and run all tests in a directory tree")

		parser.add_argument("directory", help="The directory to search")
		parser.add_argument("--pattern", "-p", default=r"\w+\_tests", metavar="PATTERN",
			help="RegEx file name pattern to search for")
		parser.add_argument("--verbose", "-v", action="store_true",
			help="Activate verbose output")

		args = parser.parse_args()

		# Sanity check and (if necessary) update args
		TestRunner._sanity_check_args(args)

		# Discover paths of all test classes
		test_class_paths = TestRunner._discover(args.directory, args.pattern)

		if not test_class_paths:
			TestRunner._print_and_exit("Found no test modules with path {} and pattern {}".format(
				args.directory, args.pattern))

		print("Found {} test modules. Running tests now...\n".format(len(test_class_paths)))

		# Run all and print the result for each
		TestRunner._run_all(test_class_paths, args.verbose)

		exit()


	@staticmethod
	def _sanity_check_args(args):
		dir_expanded = os.path.expanduser(args.directory)

		if not os.path.lexists(dir_expanded):
			TestRunner._print_and_exit("Supplied path {} does not exist.".format(dir_expanded))

		args.directory = dir_expanded

		if r"\\" in args.pattern or "/" in args.pattern:
			TestRunner._print_and_exit("Supplied pattern contains illegal directory denotement")


	@staticmethod
	def _discover(directory, pattern):
		""" Discover files fitting to the given pattern """

		all_paths = os.walk(directory)

		# Ignore hidden directories
		all_paths = [
			path
			for path in all_paths
			if not TestRunner._path_is_hidden(path[0])]

		matching_paths = []

		for path in all_paths:
			for file_name in path[2]:
				# Skip hidden files
				if file_name.startswith("."):
					continue

				# Skip non-python files
				if not file_name.endswith(".py"):
					continue

				# Skip non-matches
				if re.search(pattern, file_name) is None:
					continue

				matching_paths.append(os.path.join(path[0], file_name))

		return matching_paths


	@staticmethod
	def _run_all(test_class_paths, verbose):

		test_results = []
		not_runnable = 0

		for path in test_class_paths:
			print("-" * 30)

			module = imp.load_source("module", path)
			name = os.path.basename(path)
			try:
				module.Tests
			except AttributeError:
				print("Module <{}> does not contain expected \"Tests\" class.\nFull path: {}\n".format(
					name, path))
				not_runnable += 1
				continue

			print("Running tests in module <{}>...".format(name))
			success = TestRunner._run_test_class(module.Tests, verbose)
			test_results.append(success)

		TestRunner._print_summary(test_results, not_runnable)


	@staticmethod
	def _run_test_class(test_class, verbose):
		"""
		Runs the given test class and prints result to stdout.
		returns: True if the run was successful
		"""

		suite = unittest.TestLoader().loadTestsFromTestCase(test_class)
		result = unittest.TestResult()
		suite.run(result)

		if result.testsRun == 0:
			print("No tests found in module!")
			return True

		print("Ran {} tests - run {} successful\n".format(
				result.testsRun, "was" if result.wasSuccessful() else "NOT")
			+ "{} errors, {} failures".format(
				len(result.errors), len(result.failures)))

		TestRunner._print_if_elements(result.errors, "errors", verbose)
		TestRunner._print_if_elements(result.failures, "failures", verbose)

		return result.wasSuccessful()


	@staticmethod
	def _print_if_elements(seq, name, verbose):
		""" Check if sequence contains items and, if yes, print each """
		if not seq:
			return

		message = "-" * 5
		message += " {} occurred in the following tests ".format(
			name.capitalize())
		message += "-" * 5

		if not verbose:
			message += "\n(use --verbose/-v for detailed output)"

		print(message)

		for elem in seq:
			print("- {}".format(elem[0]))

			if verbose:
				print("{}\n".format(elem[1]))


	@staticmethod
	def _print_summary(test_results, not_runnable):
		""" Prints out a summary of all run tests and (if necessary) not runnable tests """

		number_of_tests = len(test_results)
		number_of_successes = len([x for x in test_results if x])

		print("\n" + "=" * 30)
		print("Finished running {0} test module{3}. Successful: {1}, Failed: {2}".format(
			number_of_tests, number_of_successes, number_of_tests - number_of_successes,
			TestRunner._s_if_multiple(number_of_tests)))

		if not_runnable > 0:
			print("({} test module{} could not be run.)".format(
				not_runnable, TestRunner._s_if_multiple(not_runnable)))


	### Helper methods ###


	@staticmethod
	def _path_is_hidden(path):
		path_segments = path.split(os.sep)
		hidden = any([x.startswith(".") for x in path_segments])
		return hidden


	@staticmethod
	def _s_if_multiple(number):
		return "s" if number > 1 else ""


	@staticmethod
	def _print_and_exit(text):
		print(text)
		exit()


if __name__ == "__main__":
	TestRunner.run_with_args()
