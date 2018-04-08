#!/usr/bin/env python
""" Converter """

import numpy
import sklearn.preprocessing as sk_pre

from log_entry import LogEntry
from ids.ids_entry import IdsEntry
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


	def log_entry_to_ids_entry(self, log_entry, binary=False):
		""" Convert the given LogEntry object to a IdsEntry object. """

		app_id = ids_tools.log_entry_to_app_id(log_entry)
		ndarray = self.log_entry_to_ndarray(log_entry, app_id)
		its_class = self.log_entry_to_class(log_entry)
		if binary:
			its_class = self.class_to_binary(its_class)

		ids_entry = IdsEntry(app_id, ndarray, its_class)
		return ids_entry


	def ids_entries_to_train_dict(self, ids_entries, printer):
		""" Store the given IdsEntry objects as { app_id : (X, y) }. """

		printer.prt("Dividing the data per app id...")

		app_id_datasets = {}
		for app_id in self.app_ids:
			app_id_datasets[app_id] = ([], [])

		for entry in ids_entries:
			app_id_datasets[entry.app_id][0].append(entry.vector)
			app_id_datasets[entry.app_id][1].append(entry.vclass)

		printer.prt("Done.")
		return app_id_datasets


	def log_entries_to_train_dict(self, log_entries, printer):
		""" Convert the given log entries to { app_id : (X, y) }. """

		printer.prt("Transforming the log data to trainable vectors...")
		ids_entries = [self.log_entry_to_ids_entry(e) for e in log_entries]
		printer.prt("Done.")
		return self.ids_entries_to_train_dict(ids_entries, printer)


	def log_entry_to_ndarray(self, log_entry, app_id):
		"""
		Convert the given LogEntry object to a learnable vector.
		returns: C-ordered numpy.ndarray (dense) with dtype=float64
		"""

		# We have: vin, app_id, level, log_message, gps_position, time_unix, log_id
		assert(len(log_entry.data) == 7)

		data_dict = log_entry.data
		# Discard log_id (unnecessary) and app_id (it's used for mapping to a classifier)
		# Discard VIN (we don't plan on involvin specific VINs in intrusion detection)
		# vin_int_list = self.vin_to_int_list(data_dict[LogEntry.VIN_FIELD])
		# Discard time_unix
		# time_unix = data_dict[LogEntry.TIME_UNIX_FIELD]
		# Map level to int
		level_int = self.level_mapping[data_dict[LogEntry.LEVEL_FIELD]]
		# Convert log_message to float based on app_id
		log_msg_float_list = self.log_message_to_float_list(data_dict[LogEntry.LOG_MESSAGE_FIELD], app_id)
		# Convert gps_position to float
		gps_int_list = self.gps_position_to_int_list(data_dict[LogEntry.GPS_POSITION_FIELD])

		# level int, 1-4 log message ints
		data = [level_int] + log_msg_float_list
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

		if not isinstance(the_class, int):
			raise TypeError("Expected int. Got: {}".format(type(the_class)))

		# Ensure we still have the state we expect.
		legal_labels = ids_data.get_legal_labels()
		if len(legal_labels) != 1 or self.label_int_mapping[legal_labels[0]] != 0:
			raise ValueError("Expected value has changed!")

		return the_class != 0


	def prediction_means_outlier(self, prediction):
		""" Map the given outlier prediction to a boolean 'is outlier'. """

		# Inliers are labeled 1, while outliers are labeled -1.
		if prediction == 1:
			return False
		elif prediction == -1:
			return True
		else:
			raise ValueError("Predictions can only be +1 or -1!")


	def class_to_binary(self, input_class):
		""" Convert an int class to int binary (-1/1). """

		# Inliers are labeled 1, while outliers are labeled -1.
		assert(self.prediction_means_outlier(-1))

		if self.class_means_intruded(input_class):
			return -1
		else:
			return 1


	def classes_to_binary(self, input_classes):
		""" Convert a list of int classes to int binary (-1/1). """

		output_classes = []
		for input_class in input_classes:
			output_classes.append(self.class_to_binary(input_class))
		return output_classes


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
			red, green, blue = [int(val) for val in log_message.split(",")]

			# Returns a list with 12 values
			return IdsConverter.colour_one_hot(red, green, blue)

		# Country code string like "DE" or "CH"
		if app_id == ids_data.POSE_CC:
			assert(len(log_message) == 2)

			# Returns a list with 5 values
			return IdsConverter.country_code_one_hot(log_message)

		# POI pair "type,result"
		if app_id == ids_data.POSE_POI:
			pair = log_message.split(",")
			assert(len(pair) == 2)

			type_int = self.poi_type_mapping[pair[0]]
			result_int = self.poi_result_mapping[pair[1]]

			# Make sure we only have single digits as expected
			for val in [type_int, result_int]:
				assert(val >= 0 and val <= 9)

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
	def colour_one_hot(red, green, blue):
		"""
		Do a one-hot encoding of the given colour.
		returns: A list with a 3+4+5=12 element binary encoding.
		"""

		# For expected colours, see py_turtlesim.util.Rgb
		reds = [100, 150, 255]
		greens = [0, 125, 180, 240]
		blues = [0, 100, 120, 210, 250]
		ids_tools.verify_md5(reds + greens + blues, "32b6449030a035c63654c4a11ab15eae")

		red_encoding = IdsConverter.generic_one_hot(reds, red)
		green_encoding = IdsConverter.generic_one_hot(greens, green)
		blue_encoding = IdsConverter.generic_one_hot(blues, blue)

		return red_encoding + green_encoding + blue_encoding


	@staticmethod
	def country_code_one_hot(country_code):
		"""
		Do a one-hot encoding of the given colour.
		returns: A list with a 5 element binary encoding.
		"""

		# For expected country codes, see web_api.functionality.country_code_mapper
		country_codes = ["AT", "CH", "DE", "FR", "IT"]
		ids_tools.verify_md5(country_codes, "b1d9e303bda676c3c6a61dc21e1d07c3")

		encoding = IdsConverter.generic_one_hot(country_codes, country_code)
		return encoding


	@staticmethod
	def generic_one_hot(all_values, value):
		""" Do a one-hot encoding of the given value, which is one of all_values. """

		if value not in all_values:
			raise ValueError("Given value \"{}\" is invalid! Expected one of: {}".format(value, all_values))

		binariser = sk_pre.LabelBinarizer()
		binariser.fit(all_values)
		encoding = list(
			binariser.transform([value])
			[0]
		)

		return encoding


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

		# level int
		base_len = 1
		len_key = "len"

		constraints = {}
		# 1 value (generated)
		for gen_key in ids_data.get_generators():
			constraints[gen_key] = {len_key : base_len + 1}
		# 12 values for a one-hot encoded colour
		for colr_key in ids_data.get_colours():
			constraints[colr_key] = {len_key : base_len + 12}
		# Poses all have GPS
		for pose_key in ids_data.get_poses():
			constraints[pose_key] = {len_key : base_len + 2}

		# CC
		constraints[ids_data.POSE_CC][len_key] += 5
		# type, result
		constraints[ids_data.POSE_POI][len_key] += 2
		# x, y, targ_x, targ_y
		constraints[ids_data.POSE_TSP][len_key] += 4

		expected_len = constraints[app_id][len_key]
		if len(ndarray) != expected_len:
			print(app_id)
			raise ValueError("Given ndarray has invalid length. Expected {} elements. Received: {}"
				.format(expected_len, ndarray))
