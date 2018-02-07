#!/usr/bin/env python
""" Unit and system tests for LFO and related classes """

import random
import unittest

from vin_generator import VinGenerator


class Tests(unittest.TestCase):
	""" Tests for the LaunchFileOrchestrator """

	def test_lfo_file_mode_manual_mode(self):
		# Manual mode
		# Let LFO save to disk
		# Compare files for only one namespace
		# Check turtle for manual mode
		raise NotImplementedError()
		pass


	def test_lfo_file_mode_regular_cases(self):
		# Test cases
		# Let LFO save to disk
		# Compare files for namespace count
		# Compare files for no manual mode
		raise NotImplementedError()
		pass


	def test_vin_gen_generate_none(self):
		""" Test generation of zero VINs """
		self._test_vin_gen_generate_x(0, True)
		self._test_vin_gen_generate_x(0, False)


	def test_vin_gen_generate_one(self):
		""" Test generation of one VIN """
		self._test_vin_gen_generate_x(1, True)
		self._test_vin_gen_generate_x(1, False)


	def test_vin_gen_generate_arbitrary(self):
		""" Test generation of arbitrary amounts of VINs """
		random_numbers = random.sample(range(50, 200), 50)
		for num in random_numbers:
			for vary in [True, False]:
				self._test_vin_gen_generate_x(num, vary)


	def _test_vin_gen_generate_x(self, number_of_vins, vary_plant):
		actual = VinGenerator.generate_vins(number_of_vins, vary_plant)
		self.assertEqual(len(actual), number_of_vins)


	def test_vin_gen_vary_plant(self):
		""" Test variation of plant letters """
		self._test_vin_gen_varies(True)


	def test_vin_gen_dont_vary_plant(self):
		""" Test no variation of plant letters """
		self._test_vin_gen_varies(False)


	def _test_vin_gen_varies(self, vary_plant):
		fifty_vins = VinGenerator.generate_vins(50, vary_plant)
		fifty_plant_letters = [x[:1] for x in fifty_vins]
		unique_plant_letters = set(fifty_plant_letters)
		number_of_plants = len(unique_plant_letters)
		if vary_plant:
			self.assertGreater(number_of_plants, 1)
		else:
			self.assertEqual(number_of_plants, 1)


if __name__ == "__main__":
	raise NotImplementedError("Class was built to be run by a TestRunner")
