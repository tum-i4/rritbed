#!/usr/bin/env python
""" Contains the IntrusionDefinition class """

import math
import random


class IntrusionDefinition(object):
	""" Container for an intrusion definition (corresponding to one launch file). """

	_intrusion_percentage = 0
	_intrusion_level = 0
	_intrude_turtle = True
	_intrude_generators = True
	_duplicate_vins = False

	_levels = ["easy", "med", "hard"]


	def __init__(self, intrusion_percentage, intrusion_level,
		intrude_turtle=True, intrude_generators=True, duplicate_vins=False):
		""" Ctor """

		object.__init__(self)

		self._intrusion_percentage = self._verify_percentage(intrusion_percentage)
		self._intrusion_level = self._get_level_id(intrusion_level)
		self._intrude_turtle = intrude_turtle
		self._intrude_generators = intrude_generators
		self._duplicate_vins = duplicate_vins


	@property
	def intrusion_levels(self):
		""" Get all possible intrusion level strings. """
		return self._levels


	def create_vin_tuples(self, vin_list):
		"""
		Create tuples (vin, intruded_bool) for each vin in the list.\n
		Introduce double-vin if requested.
		"""

		raise NotImplementedError()


	def _add_double_vin(self, vin_list):
		"""
		If double-vins were requested and the intrusion percentage is set to > 0,
		duplicate roughly that many VINs in the list.
		"""

		if not self._duplicate_vins or self._intrusion_percentage == 0:
			return vin_list

		number_of_vins = len(vin_list)
		number_of_duplicates = number_of_vins * 0.5 * (float(self._intrusion_percentage) / 100)

		if self._intrusion_percentage < 50:
			number_of_duplicates = math.ceil(number_of_duplicates)
		else:
			number_of_duplicates = math.floor(number_of_duplicates)

		number_of_duplicates = int(number_of_duplicates)

		# Select random indices
		duplicates_indices = random.sample(range(0, number_of_vins), number_of_duplicates * 2)

		# Take two pairs of indices and copy the VIN from one to the other
		for i in range(0, len(duplicates_indices) - 1, 2):
			first_index = duplicates_indices[i]
			second_index = duplicates_indices[i + 1]
			vin_list[second_index] = vin_list[first_index]

		return vin_list


	def _generate_intrusion_tuples(self, vin_list, rand_gen):
		"""
		Fills a list of tuples with vins from the given list and boolean flags
		according to the intrusion percentage saved in self.
		"""

		if self._intrusion_percentage == 0:
			return [(vin, False) for vin in vin_list]

		# Sample from a ten times bigger list to increase precision
		total_count = len(vin_list) * 10
		intruded_share = int(total_count * (float(self._intrusion_percentage) / 100.0))

		intrusion_choices = (
			[True for _ in range(0, intruded_share)]
			+ [False for _ in range(0, total_count - intruded_share)])

		intrusions = rand_gen.sample(intrusion_choices, len(vin_list))

		assert (len(intrusion_choices) == total_count)
		assert (len(intrusions) == len(vin_list))

		return [(vin_list[i], intrusions[i]) for i in range(0, len(vin_list))]


	def _verify_percentage(self, given_percentage):
		""" Verify the given percentage value and return it. """

		given_percentage = int(given_percentage)
		if given_percentage > 100 or given_percentage < 0:
			raise ValueError("Invalid percentage value (expecting [1,100])")

		return given_percentage


	def _get_level_id(self, intrusion_level_string):
		""" Convert the given level string to a level id. """

		assert(len(self.intrusion_levels) == 3)

		if intrusion_level_string not in self.intrusion_levels:
			raise ValueError("Invalid intrusion level string")

		return self.intrusion_levels.index(intrusion_level_string)
