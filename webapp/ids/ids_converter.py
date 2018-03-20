#!/usr/bin/env python
""" Converter """

import numpy

from log_entry import LogEntry
import ids_data
import ids_tools


class IdsConverter(object):
	""" Conversion of LogEntry objects. """

	def __init__(self):
		""" Ctor. """

		self._app_ids = ids_data.get_app_ids()
		ids_tools.verify_md5(self._app_ids, ids_data.APP_IDS_MD5)

		self._level_mapping = ids_tools.enumerate_to_dict(
			ids_data.get_levels(),
			verify_hash=ids_data.LEVEL_MAPPING_MD5)

		self._poi_type_mapping = ids_tools.enumerate_to_dict(
			ids_data.get_poi_types(),
			verify_hash=ids_data.POI_TYPE_MAPPING_MD5)

		self._poi_result_mapping = ids_tools.enumerate_to_dict(
			ids_data.get_poi_results(),
			verify_hash=ids_data.POI_RESULT_MAPPING_MD5)

		self._label_int_mapping = ids_tools.enumerate_to_dict(
			ids_data.get_labels(),
			verify_hash=ids_data.LABEL_INT_MAPPING_MD5)

		self._int_label_mapping = ids_tools.flip_dict(
			self._label_int_mapping,
			verify_hash=ids_data.INT_LABEL_MAPPING_MD5)


	def log_entry_to_ndarray(self, log_entry, app_id):
		"""
		Convert the given LogEntry object to a learnable vector.
		returns: C-ordered numpy.ndarray (dense) with dtype=float64
		"""

		# We have: vin, app_id, level, log_message, gps_position, time_unix, log_id
		assert(len(log_entry.data) == 7)

		data_dict = log_entry.data
		# Discard log_id (unnecessary) and app_id (it's used for mapping to a classifier)
		# Convert vin to float
		vin_float = self.vin_to_float(data_dict[LogEntry.VIN_FIELD])
		# Keep time_unix as is
		time_unix = data_dict[LogEntry.TIME_UNIX_FIELD]
		# Map level to int
		level_int = self._level_mapping[data_dict[LogEntry.LEVEL_FIELD]]
		# Convert gps_position to float
		gps_float = self.gps_position_to_float(data_dict[LogEntry.GPS_POSITION_FIELD], app_id)
		# Convert log_message to float based on app_id
		log_msg_float = self.log_message_to_float(data_dict[LogEntry.LOG_MESSAGE_FIELD], app_id)

		data = [log_msg_float, vin_float, level_int, time_unix]
		if gps_float is not None:
			data += [gps_float]

		result = numpy.array(
			data,
			dtype=numpy.float_,
			order="C")

		self.verify_ndarray(result, app_id)

		return result


	def log_entry_to_class(self, log_entry, multi_class):
		""" Map the given LogEntry object to a class to predict. """

		if not log_entry.intrusion:
			raise ValueError("Given LogEntry does not have a set intrusion to convert.")

		if multi_class:
			return self._label_int_mapping[log_entry.intrusion]

		legal_labels = ids_data.get_legal_labels()
		if len(legal_labels) != 1:
			raise NotImplementedError("Did not expect more than one legal label.")
		legal_mapping = self._label_int_mapping[legal_labels[0]]
		if legal_mapping != 0:
			raise ValueError("Expected a mapping to 0 for the legal label.")

		if log_entry.intrusion in legal_labels:
			return legal_mapping
		elif log_entry.intrusion in ids_data.get_intrusion_labels():
			return 1


	@staticmethod
	def vin_to_float(vin):
		""" Convert the given VIN to float(aggregate([ord(char), int(rest)])). """

		if len(vin) != 7:
			raise ValueError("Invalid VIN")

		return IdsConverter.aggregate_ints_to_float([ord(vin[0]), int(vin[1:])])


	@staticmethod
	def gps_position_to_float(gps_position, app_id):
		""" Convert the given GPS position string to (lat, lon). """

		if app_id not in ids_data.get_poses():
			return None

		# Format: lat,lon
		split = gps_position.split(",")
		if len(split) != 2:
			raise ValueError("Invalid string")

		return IdsConverter.aggregate_ints_to_float([int(split[0]), int(split[1])])


	def log_message_to_float(self, log_message, app_id):
		""" Convert the given log message to a float list based on the given app_id. """

		if app_id not in self._app_ids:
			raise ValueError("Invalid value for app_id given: {}".format(app_id))

		# Generators send "{f}"
		if app_id in ids_data.get_generators():
			return float(log_message)

		# Colour sends "{i},{i},{i}"
		if app_id in ids_data.get_colours():
			vals = [int(val) for val in log_message.split(",")]
			assert(len(vals) == 3)
			for val in vals:
				assert(val >= 0 and val <= 255)

			# Transform from [0, 255] to [1, 256] to not have zeroes
			vals = [v + 1 for v in vals]

			# Pad to ensure 12,155,1 is different from 121,55,1
			return IdsConverter.aggregate_ints_to_float(vals, pad_zeroes=3)

		# Poses
		assert(app_id in ids_data.get_poses())

		# Country code string like "DE" or "CH"
		if app_id == ids_data.POSE_CC:
			assert(len(log_message) == 2)

			ord_ints = [ord(x) for x in log_message]
			return IdsConverter.aggregate_ints_to_float(ord_ints)

		# POI pair "type,result"
		if app_id == ids_data.POSE_POI:
			pair = log_message.split(",")
			assert(len(pair) == 2)

			# Transform from [0,] to [1,] to not have zeroes
			type_int = 1 + self._poi_type_mapping[pair[0]]
			result_int = 1 + self._poi_result_mapping[pair[1]]

			# Make sure we only have single digits as expected
			for val in [type_int, result_int]:
				assert(val >= 1 and val <= 9)

			return IdsConverter.aggregate_ints_to_float([type_int, result_int])

		# Two positions as "{},{},{},{}" (start,end as x,y)
		if app_id == ids_data.POSE_TSP:
			coords = [int(coord) for coord in log_message.split(",")]
			assert(len(coords) == 4)
			for coord in coords:
				assert(coord >= 0 and coord < 500)

			# Transfrom from [0, 499] to [1, 500] to not have zeroes
			coords = [c + 1 for c in coords]

			# Pad to ensure 1,100,1,110 is different from 11,1,1,1
			return IdsConverter.aggregate_ints_to_float(coords, pad_zeroes=3)

		raise NotImplementedError("Pose type {} not implemented".format(app_id))


	@staticmethod
	def aggregate_ints_to_float(list_of_ints, pad_zeroes=None):
		""" Aggregate the given ints as float(intintint). """

		result = ""
		for i in list_of_ints:
			str_i = str(i)

			if pad_zeroes:
				assert(len(str_i) <= pad_zeroes)
				str_i = str_i.zfill(pad_zeroes)

			result += str_i

		return float(result)


	@staticmethod
	def verify_ndarray(ndarray, app_id):
		""" Verifies the given ndarray fits the app_id classifier. """

		if not isinstance(ndarray, numpy.ndarray) or ndarray.dtype != numpy.float_:
			raise ValueError("Given array is of invalid type.")

		expected_len = 4
		if app_id in ids_data.get_poses():
			expected_len += 1
		if len(ndarray) != expected_len:
			raise ValueError("Given ndarray is too short. Expected {} elements. Received: {}"
				.format(expected_len, ndarray))

		constraints = {}

		# No constraint
		for gen in ids_data.get_generators():
			constraints[gen] = lambda x: True

		# Min: 1,1,1; max: 256,256,256
		for colr in ids_data.get_colours():
			constraints[colr] = lambda x: x >= 1001001 and x <= 256256256

		# For each char: min: 65 ("A"); max: 90 ("Z")
		constraints[ids_data.POSE_CC] = lambda x: x >= 6565 and x <= 9090

		# For both ints: [1,9]
		constraints[ids_data.POSE_POI] = lambda x: x >= 11 and x <= 99

		# Min: 1,1,1,1; max: 500,500,500,500
		constraints[ids_data.POSE_TSP] = lambda x: x >= 1001001001 and x <= 500500500500

		# Check the constraint with the log_message float
		assert(constraints[app_id](ndarray[0]))


	def get_expected_classes(self, app_id, multi_class):
		""" Return a list of expected classes for the given app_id classifier. """

		labels = None
		verify_hash = None

		if not multi_class:
			return [0, 1]

		if app_id in ids_data.get_generators():
			labels = ids_data.get_labels_gens()
			verify_hash = "3e7c91c61534c25b3eb15d40d0c99a73"
		elif app_id in ids_data.get_colours():
			labels = ids_data.get_labels_colrs()
			verify_hash = "e5dce1652563eb67347003bc2f7f3e70"
		elif app_id == ids_data.POSE_CC:
			labels = ids_data.get_labels_pose_cc()
			verify_hash = "5e550fa679c1e0845320660a3c98bb6f"
		elif app_id == ids_data.POSE_POI:
			labels = ids_data.get_labels_pose_poi()
			verify_hash = "9d60b17b201114a17179334aeea66ab5"
		elif app_id == ids_data.POSE_TSP:
			labels = ids_data.get_labels_pose_tsp()
			verify_hash = "9027b46c491b3c759215fdba37a93d84"
		else:
			raise ValueError("Invalid app_id given: {}".format(app_id))

		ids_tools.verify_md5(labels, verify_hash)

		return [self._label_int_mapping[x] for x in labels]
