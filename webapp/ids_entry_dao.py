#!/usr/bin/env python
""" Convenient access to IDS entries, stored in various forms. """

import os
from enum import Enum

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
	""" Get a generator of IdsEntry objects from the given file.
	First retrieval might be slower than others. """

	if not os.path.lexists(file_path):
		raise IOError("File not found: %s" % file_path)

	raise NotImplementedError()


def save_entries(file_path, entries):
	""" Store the entries as a IDSE file. """

	if os.path.lexists(file_path):
		raise IOError("Output file exists already: %s" % file_path)

	raise NotImplementedError()


def _detect_type(first_line):
	raise NotImplementedError()


def _yield_idse_lines(yielder):
	raise NotImplementedError()


def _yield_log_lines(yielder, first_line):
	raise NotImplementedError()



# pylint: disable-msg=R0903, C0111; (Too few public methods - enum, missing class docstring - obvious)
class FileType(Enum):
	LOG_FILE = 0
	IDSE_FILE = 1
