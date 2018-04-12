#!/usr/bin/env python
""" Log file processor """

# pylint: disable-msg=C0103

import argparse
import os

from ids.dir_utils import Dir
from log_entry import LogEntry


def proc_log_string(orig_log_string):
	""" Process a line. """

	if "COLOUR" not in orig_log_string:
		return orig_log_string

	log_entry = LogEntry.from_log_string(orig_log_string)
	if log_entry.data[LogEntry.APP_ID_FIELD] != "COLOUR":
		return orig_log_string

	message = log_entry.data[LogEntry.LOG_MESSAGE_FIELD]
	r, g, b = [float(x) for x in message.split(",")]

	if any([num not in range(0, 256) for num in (r, g, b)]):
		raise RuntimeError("FIX THIS SHIT! File corrupted.")

	if (r, g, b) in [(150, 140, 200), (170, 250, 140), (120, 180, 130), (120, 180, 200)]:
		assert(log_entry.intrusion == "normal")
		return orig_log_string
	elif (r, g, b) in [(255, 0, 0), (200, 50, 50), (170, 80, 80)]:
		log_entry.set_any(intrusion="red")
		new_log_string = log_entry.get_log_string()
		return new_log_string
	else:
		raise NotImplementedError("Unexpected colour in line: %s" % orig_log_string)


def prexit(msg):
	""" Print, then exit. """
	print(msg)
	exit()


### Main program ###


p = argparse.ArgumentParser()
p.add_argument("file_path", metavar="PATH/FILE", help="Log file")
args = p.parse_args()

orig_path = os.path.expanduser(args.file_path)

if not os.path.lexists(orig_path):
	prexit("File doesn't exist")

tmp_path = orig_path + "_bak"

if os.path.lexists(tmp_path):
	prexit("%s exists" % tmp_path)

os.rename(orig_path, tmp_path)

with open(orig_path, "w") as output_file:
	for line in Dir.yield_lines(tmp_path):
		processed_string = proc_log_string(line)
		output_file.write("%s\n" % processed_string)

print("Done.")
print("Wrote to: %s" % orig_path)
print("Old file: %s" % tmp_path)
