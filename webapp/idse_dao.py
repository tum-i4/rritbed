#!/usr/bin/env python
""" Convenient access to IDS entries, stored in various forms. """

import argparse
import os
from collections import namedtuple
from enum import Enum

import numpy

from log_entry import LogEntry
from ids.dir_utils import Dir
from ids.ids_converter import IdsConverter
from ids.ids_entry import IdsEntry


#### IDSE FILE SPECIFICATION
#### (aka glorified CSV)
####
#### Ideally use the file extension as follows:
FILE_EXTENSION = "idse"
####
#### The first line in each file is the header (see CSV):
HEADER = "IDSE_V_1(APP_ID,FEATURE_COUNT,VCLASS,FEATURES)"
####
#### Each line contains, comma-separated:
#### 1   app_id : str
#### 1   feature_count : int
#### 1   vclass : [-1, 1]
#### 1-X feature : float

# Definitions
MIN_ELEMENTS_PER_LINE = 4
# Expected types of fields; the last element is the type for all remaining feature fields.
ELEMENT_TYPES = [str, int, int, float]
# Verifiers can (but don't have to be) defined; each index will be checked with the given lambda.
Verifier = namedtuple("Verifier", "element_index, verify_lambda")
VERIFIER = [
	Verifier(element_index=1, verify_lambda=lambda x: x > 0),
	Verifier(element_index=2, verify_lambda=lambda x: x in [-1, 1])
]


def yield_entries(file_path, limit=None):
	"""
	Yield IdsEntry objects from the given file. First access on log files is costly!
	*limit: Optional maximum number of entries to retrieve.
	"""

	if not os.path.lexists(file_path):
		_raise_file_doesnt_exist(file_path)

	yielder = Dir.yield_lines(file_path, limit)

	first_line = yielder.next()
	file_type = _detect_type(first_line)

	if file_type == FileType.IDSE_FILE:
		return _yield_idse_lines(yielder)
	elif file_type == FileType.LOG_FILE:
		return _read_log_lines_then_yield(yielder, first_line)
	else:
		raise NotImplementedError("File type not implemented: %s" % file_type)


def save_entries(file_path, ids_entry_generator):
	"""
	Store the entries as a IDSE file.
	returns: The file path which was stored to.
	"""

	file_path_full = add_idse_extension(file_path)

	if os.path.lexists(file_path_full):
		_raise_file_exists(file_path_full)

	if isinstance(ids_entry_generator, list):
		ids_entry_generator = (x for x in ids_entry_generator)

	first_entry = ids_entry_generator.next()

	if not isinstance(first_entry, IdsEntry):
		raise TypeError("Given elements are no IdsEntry objects!")

	lines = [
		HEADER,
		_ids_entry_to_idse_string(first_entry)
	]

	for ids_entry in ids_entry_generator:
		line = _ids_entry_to_idse_string(ids_entry)
		lines.append(line)

	Dir.write_lines(file_path_full, lines)

	return file_path_full


def convert(input_path):
	""" Convert the given file do a IDSE file. """

	if not os.path.lexists(input_path):
		_raise_file_doesnt_exist(input_path)

	output_path = add_idse_extension(input_path)

	if os.path.lexists(output_path):
		_raise_file_exists(output_path)

	save_entries(output_path, yield_entries(input_path))


def add_idse_extension(file_path):
	""" Adds the .idse extension if not present. """

	full_file_extension = "%s%s" % (os.path.extsep, FILE_EXTENSION)
	if file_path.endswith(full_file_extension):
		return file_path
	return "%s%s" % (file_path, full_file_extension)


def _detect_type(first_line):
	""" Detect the file type from the first line. """

	if first_line == HEADER:
		return FileType.IDSE_FILE

	try:
		_ = LogEntry.from_log_string(first_line)
		return FileType.LOG_FILE
	except ValueError:
		raise ValueError("Invalid file given! Can't parse:\n%s" % first_line)


def _yield_idse_lines(yielder):
	""" Yield (and verify) IDSE lines one by one from the given yielder. """

	converter = IdsConverter()

	for line in yielder:
		app_id, vector, vclass = _process_idse_line(line, converter)
		yield IdsEntry(app_id, vector, vclass)


def _process_idse_line(line, converter):
	""" Verify and convert the given IDSE line to a (app_id, vector, vclass) tuple. """

	line_elements = line.split(",")
	_verify_line_elements(line_elements)

	# Each line contains:
	# app_id : str, feature_count : int, vclass : [-1, 1], ...
	app_id = str(line_elements[0])
	feature_count = int(line_elements[1])
	vclass = int(line_elements[2])
	# ... feature_1 : float(, feature_2 : float ...)
	features = [float(x) for x in line_elements[3:]]

	if len(features) != feature_count:
		_raise_corrupt_idse_error("Invalid feature count in line! Expected: %s; Got: %s (len: %s)"
			% (feature_count, features, len(features)), reading=True)

	vector = numpy.array(features, dtype=numpy.float_, order="C")
	converter.verify_vector(vector, app_id)

	return (app_id, vector, vclass)


def _ids_entry_to_idse_string(ids_entry):
	""" Convert the given IdsEntry to a IDSE conform string. """

	app_id = ids_entry.app_id
	feature_count = len(ids_entry.vector)
	vclass = ids_entry.vclass
	features = ids_entry.vector

	line_elements = [app_id, feature_count, vclass]
	line_elements.extend(features)
	_verify_line_elements(line_elements, reading=False)

	strings = [str(x) for x in [app_id, feature_count, vclass]]
	strings += [float.__repr__(x) % x for x in features]

	line = ",".join(strings)
	return line


def _verify_line_elements(line_elements, reading=True):
	""" Verify that the given line_elements conform to our requirements. """

	if len(line_elements) < MIN_ELEMENTS_PER_LINE:
		_raise_corrupt_idse_error("Expected %s elements, got:\n%s"
			% (MIN_ELEMENTS_PER_LINE, line_elements), reading)

	line_element_types = _get_expected_types(len(line_elements))

	for line_el, exp_type in zip(line_elements, line_element_types):
		try:
			_ = exp_type(line_el)
		except ValueError:
			_raise_corrupt_idse_error("Invalid element type. Expected: \"%s\"; Got: \"%s\""
				% (type(line_el), exp_type), reading)

	for element_index, verify_lambda in VERIFIER:
		element = line_element_types[element_index](line_elements[element_index])
		if not verify_lambda(element):
			_raise_corrupt_idse_error("Data at index %s doesn't conform to verifier: %s"
				% (element_index, line_elements[element_index]), reading)


def _get_expected_types(requested_length):
	""" Get a list with expected types of the requested_length. """

	if len(ELEMENT_TYPES) != 4:
		raise NotImplementedError("Implementation has changed! Expected 4 ELEMENT_TYPES.")

	if requested_length < len(ELEMENT_TYPES):
		raise ValueError("Requested length is shorter than lenght of ELEMENT_TYPES.")

	return ELEMENT_TYPES + [ELEMENT_TYPES[3]] * (requested_length - len(ELEMENT_TYPES))


def _read_log_lines_then_yield(yielder, first_line):
	""" Read all provided log lines from the given yielder. """

	first_entry = LogEntry.from_log_string(first_line)
	log_entries = [first_entry]
	for line in yielder:
		log_entry = LogEntry.from_log_string(line)
		log_entries.append(log_entry)

	ids_entry_dict = IdsConverter().log_entries_to_ids_entries_dict(log_entries)

	for _, app_entries in ids_entry_dict.items():
		for ids_entry in app_entries:
			yield ids_entry


### Errors ###


def _raise_file_doesnt_exist(file_path):
	""" Raise an error for a missing file. """
	raise IOError("[IDSE DAO] File not found: %s" % file_path)


def _raise_file_exists(file_path):
	""" Raise an error for an existing file. """
	raise IOError("[IDSE DAO] File exists already: %s" % file_path)


def _raise_corrupt_idse_error(message, reading):
	prefix = "[IDSE DAO] "
	if reading:
		prefix += "Possibly corrupt IDSE file!"
	raise IOError("%s %s" % (prefix, message))



# pylint: disable-msg=R0903, C0111; (Too few public methods - enum, missing class docstring - obvious)
class FileType(Enum):
	LOG_FILE = 0
	IDSE_FILE = 1



### Main ###

if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		PARSER.add_argument("mode", choices=["convert"])
		PARSER.add_argument("file_path", help="Log file")
		ARGS = PARSER.parse_args()
		if ARGS.mode == "convert":
			print("Converting...")
			convert(ARGS.file_path)
			print("Done.")
		else:
			print("Doing nothing...")
	except KeyboardInterrupt:
		pass
