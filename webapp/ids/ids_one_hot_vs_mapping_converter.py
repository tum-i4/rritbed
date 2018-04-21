#!/usr/bin/env python
""" One-hot encoding vs. mapping """

import numpy

from ids_converter import IdsConverter
import ids_data
import ids_tools


class OneHotVsMappingConverter(IdsConverter):

	def __init__(self):
		super(OneHotVsMappingConverter, self).__init__()

		## Verifier data ##
		# 1 for a binarised level (only two options)
		base_len = 1
		self._len_key = "len"

		self._vector_constraints = {}
		# 1 value (generated)
		for gen_key in ids_data.get_generators():
			self._vector_constraints[gen_key] = {self._len_key : base_len + 1}
		# 3 values for a split colour, 2 values for the position
		for colr_key in ids_data.get_colours():
			self._vector_constraints[colr_key] = {self._len_key : base_len + 5}
		# Poses all have GPS
		for pose_key in ids_data.get_poses():
			self._vector_constraints[pose_key] = {self._len_key : base_len + 2}

		# CC: One mapping
		self._vector_constraints[ids_data.POSE_CC][self._len_key] += 1
		# POI: Two mappings
		self._vector_constraints[ids_data.POSE_POI][self._len_key] += 2
		# TSP: x, y, targ_x, targ_y
		self._vector_constraints[ids_data.POSE_TSP][self._len_key] += 4


	def encode_log_messages(self, app_id, log_messages):
		"""
		Either just convert the data or do convert to mapping
		returns: Two-dimensional numpy.ndarray
		"""

		# "{f}" -> float(x)
		if app_id in ids_data.get_generators():
			return numpy.array([[float(log_message)] for log_message in log_messages])

		# "{i},{i},{i}" -> int(x), int(y), int(z)
		if app_id in ids_data.get_colours():
			colours = [[int(val) for val in msg.split(",")] for msg in log_messages]
			return numpy.array(colours)

		# "DE" -> 3
		if app_id == ids_data.POSE_CC:
			country_code_mapping = ids_tools.enumerate_to_dict(
				ids_data.get_country_codes(),
				verify_hash="8ef55ff8ba5ce289a3d2a689edbaa423")
			mapped = [[country_code_mapping[c]] for c in log_messages]
			return numpy.array(mapped)

		# "type,result" -> 1,7
		if app_id == ids_data.POSE_POI:
			poi_type_mapping = ids_tools.enumerate_to_dict(
				ids_data.get_poi_types(),
				verify_hash="f2fba0ed17e382e274f53bbcb142565b")

			poi_result_mapping = ids_tools.enumerate_to_dict(
				ids_data.get_poi_results(),
				verify_hash="dd1c18c7188a48a686619fef8007fc64")

			poi_pairs = [msg.split(",") for msg in log_messages]
			poi_result = [[poi_type_mapping[a], poi_result_mapping[b]] for (a, b) in poi_pairs]

			return numpy.array(poi_result)

		# "{x},{y},{x},{y}" -> int(x),int(y),int(x),int(y)
		if app_id == ids_data.POSE_TSP:
			coords_rows = [[int(coord) for coord in msg.split(",")] for msg in log_messages]
			assert(len(coords_rows[0]) == 4)
			for coord in coords_rows[0]:
				assert(coord >= 0 and coord < 500)

			return numpy.array(coords_rows)

		raise NotImplementedError("App ID {} not implemented".format(app_id))


	def encode_positions(self, positions):
		"""
		Given: strings "x,y"
		returns: numpy.ndarray with "" -> None, "x,y" -> [int(x),int(y)]
		"""

		encoded_positions = []

		for position in positions:
			if not position:
				encoded_positions.append(None)
				return
			split = position.split(",")
			if len(split) != 2:
				raise ValueError("Invalid string")
			encoded_positions.append([int(x) for x in split])

		return numpy.array(encoded_positions)
