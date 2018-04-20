#!/usr/bin/env python
""" One-hot encoding vs. mapping """

import numpy

from ids_converter import IdsConverter
import ids_data
import ids_tools


class OneHotVsMappingConverter(IdsConverter):

	def __init__(self):
		super(OneHotVsMappingConverter, self).__init__(self)


	@staticmethod
	def encode_log_messages(app_id, log_messages):
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
				verify_hash="b1d9e303bda676c3c6a61dc21e1d07c3")
			mapped = [country_code_mapping[c] for c in log_messages]
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


	@staticmethod
	def encode_positions(positions):
		"""
		Given: strings "x,y"
		returns: numpy.ndarray with "" -> None, "x,y" -> [int(x),int(y)]
		"""

		encoded_positions = []

		for position in positions:
			if not position:
				encoded_positions.append(None)
			split = position.split(",")
			if len(split) != 2:
				raise ValueError("Invalid string")
			encoded_positions.append([int(x) for x in split])

		return numpy.array(encoded_positions)
