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


	def log_entries_to_ids_entries_dict(self, log_entries, binary=True):
		""" Convert the given LogEntry objects to a { app_id : IdsEntrys } dict. """

		log_entries_per_app_id = ids_tools.empty_app_id_to_list_dict()

		for log_entry in log_entries:
			app_id = ids_tools.log_entry_to_app_id(log_entry)
			log_entries_per_app_id[app_id].append(log_entry)

		ids_entries_per_app_id = ids_tools.empty_app_id_to_list_dict()
		for app_id in log_entries_per_app_id:
			ids_entries = self.log_entries_to_ids_entries(app_id, log_entries, binary)
			ids_entries_per_app_id[app_id] = ids_entries

		return ids_entries_per_app_id


	def log_entries_to_ids_entries(self, app_id, log_entries, binary):
		""" Convert the given LogEntry objects to IdsEntry objects for this app_id. """

		app_ids = [ids_tools.log_entry_to_app_id(log_entry) for log_entry in log_entries]

		if any([a != app_id for a in app_ids]):
			raise ValueError("Given elements are not all of the expected app type: {}".format(app_id))

		vectors = self.log_entries_to_vectors(app_id, log_entries)
		vclasses = [self.log_entry_to_class(log_entry, binary) for log_entry in log_entries]

		ids_entries = []
		for app_id, vector, vclass in zip(app_ids, vectors, vclasses):
			ids_entry = IdsEntry(app_id, vector, vclass)
			ids_entries.append(ids_entry)

		return ids_entries


	def ids_entries_to_X_y(self, app_id, ids_entries):
		""" Convert the given IdsEntry objects to (X, y). """

		# pylint: disable-msg=C0103; (Invalid variable name)
		X = []
		y = []

		for ids_entry in ids_entries:
			if ids_entry.app_id != app_id:
				raise ValueError("Given entries don't conform to the expected app_id!")

			X.append(ids_entry.vector)
			y.append(ids_entry.vclass)

		return (X, y)


	def log_entries_to_train_dict(self, log_entries, printer):
		""" Convert the given log entries to { app_id : (X, y) }. """

		printer.prt("Transforming the log data to trainable vectors...")
		ids_entries = [self.log_entry_to_ids_entry(e) for e in log_entries]
		printer.prt("Done.")
		return self.ids_entries_to_train_dict(ids_entries, printer)


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


	def log_entries_to_vectors(self, app_id, log_entries):
		"""
		Convert the given LogEntry objects to learnable vectors.
		returns: C-ordered numpy.ndarray (dense) with dtype=float64
		"""

		# We have: vin, app_id, level, log_message, gps_position, time_unix, log_id
		assert(len(log_entries[0].data) == 7)

		# Discard log_id (unnecessary) and app_id (it's used for mapping to a classifier)
		# Discard VIN (we don't plan on involvin specific VINs in intrusion detection)
		# vin_int_list = self.vin_to_int_list(data_dict[LogEntry.VIN_FIELD])
		# Discard time_unix
		levels = []
		log_messages = []
		gps_positions = []

		for log_entry in log_entries:
			data_dict = log_entry.data

			levels.append(data_dict[LogEntry.LEVEL_FIELD])
			log_messages.append(data_dict[LogEntry.LOG_MESSAGE_FIELD])
			gps_positions.append(data_dict[LogEntry.GPS_POSITION_FIELD])

		# One-hot encoding of levels -> [0, 1]
		enc_levels_array = IdsConverter.encode_levels(levels)
		# Conversion (data gens) or one-hot encoding of log messages -> [0, 1, ...]
		enc_log_messages_array = IdsConverter.encode_log_messages(app_id, log_messages)
		# Convert GPS positions to None or (x, y)
		enc_gps_positions_array = IdsConverter.encode_gps_positions(gps_positions)

		vectors = []

		for enc_lvl, enc_msg, enc_gps in (
			zip(enc_levels_array, enc_log_messages_array, enc_gps_positions_array)):

			# 2 level ints, 1-12 log message floats or ints
			data = list(enc_lvl) + list(enc_msg)
			# 0/2 GPS ints
			if enc_gps is not None:
				data += enc_gps

			ndarray = numpy.array(
				data,
				dtype=numpy.float_,
				order="C")

			self.verify_ndarray(ndarray, app_id)

			vectors.append(ndarray)

		return vectors


	def log_entry_to_class(self, log_entry, binary):
		""" Map the given LogEntry object to a class to predict. """

		if not log_entry.intrusion:
			raise ValueError("Given LogEntry does not have a set intrusion to convert.")

		its_class = self.label_int_mapping[log_entry.intrusion]

		if binary:
			its_class = self.class_to_binary(its_class)

		return its_class


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
	def encode_levels(levels):
		"""
		Do a one-hot encoding of the given colour.
		returns: Two-dimensional numpy.ndarray with a 2 element binary encoding per row.
		"""

		# For expected levels, see web_api.log_entry.LogEntry
		expected_levels = ["DEBUG", "ERROR"]
		ids_tools.verify_md5(expected_levels, "7692bbdba09aa7f2c9a15ca0e9a654cd")

		encoded_levels = IdsConverter.generic_one_hot(expected_levels, levels)
		return encoded_levels


	@staticmethod
	def encode_log_messages(app_id, log_messages):
		"""
		Either just convert the data (data generators) or do a one-hot encoding of the log message.
		returns: Two-dimensional numpy.ndarray with either 1 float value, 4 int values
		or up to 12 binary encoded values per row.
		"""

		# Generators send "{f}"
		if app_id in ids_data.get_generators():
			# Return list with value
			return numpy.array([float(log_message) for log_message in log_messages])

		# Colour sends "{i},{i},{i}"
		if app_id in ids_data.get_colours():
			colours = [[int(val) for val in msg.split(",")] for msg in log_messages]

			# Returns a list with 12 values
			return IdsConverter.colours_one_hot(colours)

		# Country code string like "DE" or "CH"
		if app_id == ids_data.POSE_CC:
			assert(len(log_messages[0]) == 2)

			# Returns a list with 5 values
			return IdsConverter.country_codes_one_hot(log_messages)

		# POI pair "type,result"
		if app_id == ids_data.POSE_POI:
			# Split and check split size
			poi_pairs = [[(t, r) for t, r in msg.split(",")] for msg in log_messages]

			# Returns a list with 10 values
			return IdsConverter.poi_pairs_one_hot(poi_pairs)

		# Two positions as "{},{},{},{}" (start,end as x,y)
		if app_id == ids_data.POSE_TSP:
			coords_list = [[int(coord) for coord in msg.split(",")] for msg in log_messages]
			assert(len(coords_list[0]) == 4)
			for coord in coords_list[0]:
				assert(coord >= 0 and coord < 500)

			# Return list of 4 coordinates
			return coords_list

		raise NotImplementedError("App ID {} not implemented".format(app_id))


	@staticmethod
	def colours_one_hot(colours):
		"""
		Do a one-hot encoding of the given colours.
		returns: A two-dimensional numpy.ndarray with a 3+4+5=12 element binary encoding per row.
		"""

		# For expected colours, see py_turtlesim.util.Rgb
		reds = [100, 150, 255]
		greens = [0, 125, 180, 240]
		blues = [0, 100, 120, 210, 250]
		ids_tools.verify_md5(reds + greens + blues, "32b6449030a035c63654c4a11ab15eae")

		colours_array = numpy.array(colours)

		red_encodings = IdsConverter.generic_one_hot(reds, colours_array[:, 0])
		green_encodings = IdsConverter.generic_one_hot(greens, colours_array[:, 1])
		blue_encodings = IdsConverter.generic_one_hot(blues, colours_array[:, 2])

		encodings = numpy.concatenate((red_encodings, green_encodings, blue_encodings), axis=1)
		return encodings


	@staticmethod
	def country_codes_one_hot(country_codes):
		"""
		Do a one-hot encoding of the given country codes.
		returns: A two-dimensional numpy.ndarray with a 5 element binary encoding per row.
		"""

		# For expected country codes, see web_api.functionality.country_code_mapper
		expected_cc = ["AT", "CH", "DE", "FR", "IT"]
		ids_tools.verify_md5(expected_cc, "b1d9e303bda676c3c6a61dc21e1d07c3")

		encodings = IdsConverter.generic_one_hot(expected_cc, country_codes)
		return encodings


	@staticmethod
	def poi_pairs_one_hot(poi_pairs):
		"""
		Do a one-hot encoding of the given POI pairs.
		returns: A two-dimensional numpy.ndarray with a 4+6=10 element binary encoding per row.
		"""

		# For expected POI types, see turtlesim_expl.pipes.pose_processor
		expected_types = ["restaurant", "gas station", "private home", "nsa hq"]
		ids_tools.verify_md5(expected_types, "d36c52a68115ddd49db3f8b8fb818798")
		# For expected POI results, see web_api.functionality.poi_mapper
		exptected_results = ["Italian", "German", "French", "Total", "Shell", "Aral"]
		ids_tools.verify_md5(exptected_results, "ce5b1e82cc76efb5835d6f78e7cd3af4")

		poi_pairs_array = numpy.array(poi_pairs)

		types_encodings = IdsConverter.generic_one_hot(expected_types, poi_pairs_array[:, 0])
		results_encodings = IdsConverter.generic_one_hot(exptected_results, poi_pairs_array[:, 1])

		encodings = numpy.concatenate((types_encodings, results_encodings), axis=1)
		return encodings


	@staticmethod
	def encode_gps_positions(gps_positions):
		"""
		Convert the given "x,y" GPS position strings to (x, y) or None.
		returns: A two-dimensional numpy.ndarray with a result (tuple or None) per row.
		"""

		encoded_positions = [IdsConverter.gps_position_to_int_list(gps_pos) for gps_pos in gps_positions]
		return numpy.array(encoded_positions)


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


	@staticmethod
	def generic_one_hot(expected_values, values):
		"""
		Do a one-hot encoding of the given values, which are one of expected_values.
		returns: A two-dimensional numpy.ndarray with one encoding per row.
		"""

		if any([value not in expected_values for value in values]):
			raise ValueError("Given value \"{}\" is invalid! Expected one of: {}"
				.format(value, expected_values))

		binariser = sk_pre.LabelBinarizer()
		binariser.fit(expected_values)
		encodings = list(binariser.transform(values))

		return encodings


	### Verification ###


	def verify_ndarray(self, ndarray, app_id):
		""" Verifies the given ndarray fits the app_id classifier. """

		if not isinstance(ndarray, numpy.ndarray) or ndarray.dtype != numpy.float_:
			raise ValueError("Given array is of invalid type.")

		if app_id not in self.app_ids:
			raise ValueError("Invalid app_id: {}".format(app_id))

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
