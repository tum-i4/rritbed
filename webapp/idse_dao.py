#!/usr/bin/env python
""" Convenient access to IDS entries, stored in various forms. """

import os
from enum import Enum

from log_entry import LogEntry
from ids.dir_utils import Dir


#### IDSE FILE SPECIFICATION
#### (aka glorified CSV)
####
#### The first line in each file is the header (see CSV):
HEADER = "APP_ID,FEATURE_COUNT,VCLASS,FEATURES"
####
#### Each line contains, comma-separated:
#### 1   app_id : str
#### 1   feature_count : int
#### 1   vclass : [-1, 1]
#### 1-X feature : float


def get_entries(file_path, limit=None):
	""" Return up IdsEntry objects from the given file.
	*limit: Optional maximum number of entries to retrieve. """

	if not os.path.lexists(file_path):
		raise IOError("File not found: %s" % file_path)

	yielder = Dir.yield_lines(file_path, limit)

	first_line = yielder.next()
	file_type = _detect_type(first_line)

	if file_type == FileType.IDSE_FILE:
		_yield_idse_lines(yielder)
	elif file_type == FileType.LOG_FILE:
		_read_log_lines(yielder, first_line)
	else:
		raise NotImplementedError("File type not implemented: %s" % file_type)


def save_entries(file_path, entries):
	""" Store the entries as a IDSE file. """

	if os.path.lexists(file_path):
		raise IOError("Output file exists already: %s" % file_path)

	raise NotImplementedError()


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
	raise NotImplementedError()


def _read_log_lines(yielder, first_line):
	raise NotImplementedError()



# pylint: disable-msg=R0903, C0111; (Too few public methods - enum, missing class docstring - obvious)
class FileType(Enum):
	LOG_FILE = 0
	IDSE_FILE = 1
