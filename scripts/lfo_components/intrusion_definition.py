#!/usr/bin/env python
""" Contains the IntrusionDefinition class """

import math
import random


class IntrusionDefinition(object):
	""" Container for an intrusion definition (corresponding to one launch file). """

	_LEVELS = ["easy", "med", "hard"]

	_EASY_INTELLIGENCE = ["stay", "dont-move"]
	_TURTLE_INTELLIGENCE_CHOICES = {
		0 : _EASY_INTELLIGENCE,
		1 : _EASY_INTELLIGENCE,
		2 : _EASY_INTELLIGENCE
	}

	# Check DistributionGenerator if these are correct
	_BLUNT_INTRUSIONS = ["zeroes", "huge-error"]
	_GENERATOR_INTRUSION_CHOICES = {
		0 : _BLUNT_INTRUSIONS,
		1 : _BLUNT_INTRUSIONS,
		2 : _BLUNT_INTRUSIONS
	}


	# pylint: disable-msg=R0913; (Too many arguments)
	def __init__(self, intrusion_percentage, intrusion_level,
		intrude_turtle=True, intrude_generators=True, duplicate_vins=False):
		""" Ctor """

		object.__init__(self)

		self._intrusion_percentage = self._verify_percentage(intrusion_percentage)
		self._intrusion_level = self._get_level_id(intrusion_level)
		self._intrude_turtle = intrude_turtle
		self._intrude_generators = intrude_generators
		self._duplicate_vins = duplicate_vins


	@staticmethod
	def get_intrusion_levels():
		""" Get all possible intrusion level strings. """
		return IntrusionDefinition._LEVELS


	def create_vin_tuples(self, vin_list):
		""" Create tuples (vin, intruded_bool) for each vin in the list.\n
		Introduce double-vin if requested. """

		vin_list = self._add_double_vin(vin_list)
		vin_tuples = self._generate_intrusion_tuples(vin_list)
		return vin_tuples


	def get_turtle_intelligence(self, intruded, legal_choices):
		""" Return a list of possible intelligence options based on the specified intrusion level. """

		if not intruded or not self._intrude_turtle:
			return random.choice(legal_choices)

		intrusion_choices = IntrusionDefinition._TURTLE_INTELLIGENCE_CHOICES[self._intrusion_level]

		# Easy and medium: Always intrude turtle
		if self._intrusion_level in [0, 1]:
			pass
		# Hard: Intrude only in 50 % of cases: Pre-select one intrusion and one legal choice
		elif self._intrusion_level == 2:
			intrusion_choices = [random.choice(legal_choices), random.choice(intrusion_choices)]
		else:
			raise NotImplementedError("get_turtle_intelligence(): Missing intrusion level")

		return random.choice(intrusion_choices)


	def get_turtle_args(self, intruded):
		""" Return args or None for the py_turtlesim based on the specified intrusion level. """

		return self._get_arg(condition=(intruded and self._intrude_turtle))


	def get_logger_arg(self, intruded):
		""" Return the intrusion arg or empty string based on the specified intrusion level. """

		arg = self._get_arg(condition=intruded)
		return (" " + arg) if arg is not None else ""


	def _get_arg(self, condition):
		""" Return the intrusion arg or None based on the given boolean. """

		if not condition:
			return None

		return "--intrusion " + IntrusionDefinition._LEVELS[self._intrusion_level]


	def create_generator_tuples(self, intruded, selected_generators):
		""" Create tuples (selected_generator, intrusion_mode) for each generator in the list. """

		generator_tuples = [(generator, None) for generator in selected_generators]

		if not intruded or not self._intrude_generators:
			return generator_tuples

		choices = IntrusionDefinition._GENERATOR_INTRUSION_CHOICES[self._intrusion_level]
		all_indices = list(range(0, len(generator_tuples)))
		number_of_intruded_generators = 0

		# Easy: Intrude all generators
		if self._intrusion_level == 0:
			number_of_intruded_generators = len(all_indices)
		# Medium: Intrude about 50 % of generators, at least one
		elif self._intrusion_level == 1:
			# Rounding up to always have >= 1
			number_of_intruded_generators = int(math.ceil((len(all_indices) / 2.0)))
		# Hard: Intrude about 30 % of generators, can be none
		elif self._intrusion_level == 2:
			number_of_intruded_generators = int(math.floor((len(all_indices) / 3.0)))
		else:
			raise NotImplementedError("create_generator_tuples(): Missing intrusion level")

		# Sample random items to be intruded into
		intruded_indices = random.sample(all_indices, number_of_intruded_generators)

		# Select random intrusions for the selected intruded generators
		for index in intruded_indices:
			generator = generator_tuples[index][0]
			generator_tuples[index] = (generator, random.choice(choices))

		return generator_tuples


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


	def _generate_intrusion_tuples(self, vin_list):
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

		intrusions = random.sample(intrusion_choices, len(vin_list))

		assert (len(intrusion_choices) == total_count)
		assert (len(intrusions) == len(vin_list))

		return [(vin_list[i], intrusions[i]) for i in range(0, len(vin_list))]


	@staticmethod
	def _verify_percentage(given_percentage):
		""" Verify the given percentage value and return it. """

		given_percentage = int(given_percentage)
		if given_percentage > 100 or given_percentage < 0:
			raise ValueError("Invalid percentage value (expecting [1,100])")

		return given_percentage


	@staticmethod
	def _get_level_id(intrusion_level_string):
		""" Convert the given level string to a level id. """

		levels = IntrusionDefinition._LEVELS
		assert(len(levels) == 3)

		if intrusion_level_string not in levels:
			raise ValueError("Invalid intrusion level string")

		return levels.index(intrusion_level_string)
