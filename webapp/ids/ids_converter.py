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

		self.app_ids = ids_data.get_app_ids()
		ids_tools.verify_md5(self.app_ids, "cacafa61f61b645c279954952ac6ba8f")

		self.level_mapping = ids_tools.enumerate_to_dict(
			ids_data.get_levels(),
			verify_hash="49942f0268aa668e146e533b676f03d0")

		self.poi_type_mapping = ids_tools.enumerate_to_dict(
			ids_data.get_poi_types(),
			verify_hash="f2fba0ed17e382e274f53bbcb142565b")

		self.poi_result_mapping = ids_tools.enumerate_to_dict(
			ids_data.get_poi_results(),
			verify_hash="dd1c18c7188a48a686619fef8007fc64")

		self.label_int_mapping = ids_tools.enumerate_to_dict(
			ids_data.get_labels(),
			verify_hash="69a262192b246d16e8411b6db06e237b")


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
		vin_int_list = self.vin_to_int_list(data_dict[LogEntry.VIN_FIELD])
		# Discard time_unix
		# time_unix = data_dict[LogEntry.TIME_UNIX_FIELD]
		# Map level to int
		level_int = self.level_mapping[data_dict[LogEntry.LEVEL_FIELD]]
		# Convert gps_position to float
		gps_int_list = self.gps_position_to_int_list(data_dict[LogEntry.GPS_POSITION_FIELD])
		# Convert log_message to float based on app_id
		log_msg_float_list = self.log_message_to_float_list(data_dict[LogEntry.LOG_MESSAGE_FIELD], app_id)

		# 2 VIN ints, level int, 1-4 log message ints
		data = vin_int_list + [level_int] + log_msg_float_list
		# 0/2 GPS ints
		if gps_int_list is not None:
			data += gps_int_list

		result = numpy.array(
			data,
			dtype=numpy.float_,
			order="C")

		self.verify_ndarray(result, app_id)

		return result


	def log_entry_to_class(self, log_entry):
		""" Map the given LogEntry object to a class to predict. """

		if not log_entry.intrusion:
			raise ValueError("Given LogEntry does not have a set intrusion to convert.")

		return self.label_int_mapping[log_entry.intrusion]


	def class_means_intruded(self, the_class):
		""" Map the given class to a boolean 'is intruded'. """
		return the_class != 0


	def get_expected_classes(self, app_id):
		""" Return a list of expected classes for the given app_id classifier. """

		labels = None
		verify_hash = None

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

		return [self.label_int_mapping[x] for x in labels]


	### Conversions ###


	@staticmethod
	def vin_to_int_list(vin):
		""" Convert the given VIN to [ord(char), int(rest)]. """

		if len(vin) != 7:
			raise ValueError("Invalid VIN")

		return [ord(vin[0]), int(vin[1:])]


	@staticmethod
	def gps_position_to_int_list(gps_position):
		""" Convert the given GPS position string to (lat, lon). """

		if not gps_position:
			return None

		# Format: lat,lon
		split = gps_position.split(",")
		if len(split) != 2:
			raise ValueError("Invalid string")

		return [int(split[0]), int(split[1])]


	def log_message_to_float_list(self, log_message, app_id):
		""" Convert the given log message to a float list based on the given app_id. """

		if app_id not in self.app_ids:
			raise ValueError("Invalid value for app_id given: {}".format(app_id))

		# Generators send "{f}"
		if app_id in ids_data.get_generators():
			# Return list with value
			return [float(log_message)]

		# Colour sends "{i},{i},{i}"
		if app_id in ids_data.get_colours():
			vals = [int(val) for val in log_message.split(",")]
			assert(len(vals) == 3)
			for val in vals:
				assert(val >= 0 and val <= 255)

			# Return list with the three values
			return vals

		# Country code string like "DE" or "CH"
		if app_id == ids_data.POSE_CC:
			assert(len(log_message) == 2)

			ord_ints = [ord(x) for x in log_message]
			# Return list with the chars as one aggregated int
			return [IdsConverter.aggregate_ints(ord_ints)]

		# POI pair "type,result"
		if app_id == ids_data.POSE_POI:
			pair = log_message.split(",")
			assert(len(pair) == 2)

			type_int = self.poi_type_mapping[pair[0]]
			result_int = self.poi_result_mapping[pair[1]]

			# Make sure we only have single digits as expected
			for val in [type_int, result_int]:
				assert(val >= 1 and val <= 9)

			# Return list with mapped type and result
			return [type_int, result_int]

		# Two positions as "{},{},{},{}" (start,end as x,y)
		if app_id == ids_data.POSE_TSP:
			coords = [int(coord) for coord in log_message.split(",")]
			assert(len(coords) == 4)
			for coord in coords:
				assert(coord >= 0 and coord < 500)

			# Return list of coordinates
			return coords

		raise NotImplementedError("Pose type {} not implemented".format(app_id))


	@staticmethod
	def aggregate_ints(list_of_ints, pad_zeroes=None):
		""" Aggregate the given ints as int(intintint). """

		result = ""
		for i in list_of_ints:
			str_i = str(i)

			if pad_zeroes:
				assert(len(str_i) <= pad_zeroes)
				str_i = str_i.zfill(pad_zeroes)

			result += str_i

		return int(result)


	### Verification ###


	@staticmethod
	def verify_ndarray(ndarray, app_id):
		""" Verifies the given ndarray fits the app_id classifier. """

		if not isinstance(ndarray, numpy.ndarray) or ndarray.dtype != numpy.float_:
			raise ValueError("Given array is of invalid type.")

		# 2 VIN ints, level int
		base_len = 3
		len_key = "len"

		constraints = {}
		# 1 value (generated)
		constraint_generators = {len_key : base_len + 1}
		for gen_key in ids_data.get_generators():
			constraints[gen_key] = constraint_generators
		# 3 values for each colour dimension
		constraint_colours = {len_key : base_len + 3}
		for colr_key in ids_data.get_colours():
			constraints[colr_key] = constraint_colours

		# CC
		constraints[ids_data.POSE_CC] = {len_key : base_len + 1}
		# type, result
		constraints[ids_data.POSE_POI] = {len_key : base_len + 2}
		# x, y, targ_x, targ_y
		constraints[ids_data.POSE_TSP] = {len_key : base_len + 4}

		expected_len = constraints[app_id][len_key]
		if len(ndarray) != expected_len:
			raise ValueError("Given ndarray is too short. Expected {} elements. Received: {}"
				.format(expected_len, ndarray))
