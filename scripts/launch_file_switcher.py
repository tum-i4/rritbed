#!/usr/bin/env python
""" Launch File Switcher """

import argparse
import os
import random

def info(msg):
	print("[Info] %s" % msg)

def line_warning(line_no, the_line, warning):
	print("\n[Warning] %s:" % warning)
	print("[%s] %s\n" % (line_no, the_line.strip()))

def err_exit(msg):
	print("[Error] %s" % msg)
	exit()

def index_is_commented(item, ln):
	opener = "<!--"
	closer = "-->"
	if opener not in ln and closer not in ln:
		return False

	idx = ln.index(item)

	index_open = 0
	index_close = len(ln) - 1

	if opener in ln:
		index_open = ln.index(opener)
	if closer in ln:
		index_close = ln.index(closer)

	return index_open < idx and idx < index_close

#### Main program ####

MODES = ["easy", "med", "hard"]

PARSER = argparse.ArgumentParser()
PARSER.add_argument("file_path", help="The launch file path")
PARSER.add_argument("mode", choices=MODES, help="Target mode of the new file.")
PARSER.add_argument("--replace-in-comments", "-c", action="store_true", help="Also check comments.")
PARSER.add_argument("--dry-run", action="store_true", help="Just try without writing.")
ARGS = PARSER.parse_args()

if not os.path.lexists(ARGS.file_path):
	err_exit("Input file doesn't exist.")

MODE = ARGS.mode
PATH = os.path.expanduser(ARGS.file_path)
NAME = os.path.basename(PATH)

NEW_NAME = ""

FOUND = False

for m in MODES:
	if m in NAME:
		NEW_NAME = NAME.replace(m, MODE)
		FOUND = True
		break

RANDOM_NAME = not FOUND or MODE in NAME

if not FOUND:
	info("No mode in file name.")
elif MODE in NAME:
	info("Selected mode is already part of the given file's name.")

if RANDOM_NAME:
	info("Will use random string.")
	NEW_NAME = ("%s_converted-by-lf-switcher_%s_%s" %
		(NAME, MODE, "".join([str(x) for x in random.sample(range(0, 20), 5)]))
	)

info("Using target file name \"%s\"\n" % NEW_NAME)

NEW_PATH = os.path.join(os.path.dirname(PATH), NEW_NAME)

if os.path.lexists(NEW_PATH):
	err_exit("Target file exists! Try again or remove.")

NEW_FILE_LINES = []
EXPECTED_COUNT = 0

REPLACED_INTELLIGENCE_MODES = []
REPLACED_MODES = []

with open(PATH, "r") as ORIG_FILE:

	REPLACERS = [md for md in MODES if md != MODE]

	for index, line in enumerate(ORIG_FILE.readlines()):
		EXPECTED_COUNT += 1

		HAS_MODE = False
		IS_COMMENT = True

		for md in MODES:
			if md in line:
				HAS_MODE |= True
				IS_COMMENT &= index_is_commented(md, line)

		if (not HAS_MODE
			or IS_COMMENT and not ARGS.replace_in_comments):
			NEW_FILE_LINES.append(line)
			continue

		NEW_LINE = line
		LINE_NO = index + 1

		if not any([i in NEW_LINE for i in ["--intrusion", "--intrusion-level"]]):
			if not IS_COMMENT:
				line_warning(LINE_NO, NEW_LINE,
					"Found identifier in line, but no expected flag")

		if "intelligence" in NEW_LINE and "stay" in NEW_LINE:
			line_warning(LINE_NO, NEW_LINE, "Found incorrect intelligence mode \"stay\"")
			NEW_LINE = NEW_LINE.replace("intelligence stay", "intelligence return")
			REPLACED_INTELLIGENCE_MODES.append(LINE_NO)

		for rp in REPLACERS:
			if rp in NEW_LINE:
				NEW_LINE = NEW_LINE.replace(rp, MODE)
				REPLACED_MODES.append(LINE_NO)

		NEW_FILE_LINES.append(NEW_LINE)

info("Done. Analysing...")
if not REPLACED_INTELLIGENCE_MODES and not REPLACED_MODES:
	info("No replacements necessary. No new file created.")
	exit()

if len(NEW_FILE_LINES) != EXPECTED_COUNT:
	raise RuntimeError("Incorrect line count")

if REPLACED_MODES:
	info("Replacing intrusion levels in %s lines: %s" %
		(len(REPLACED_MODES), REPLACED_MODES))
if REPLACED_INTELLIGENCE_MODES:
	info("Replacing incorrect intelligence modes in %s lines: %s" %
		(len(REPLACED_INTELLIGENCE_MODES), REPLACED_INTELLIGENCE_MODES))

if not ARGS.dry_run:
	with open(NEW_PATH, "w") as NEW_FILE:
		for line in NEW_FILE_LINES:
			NEW_FILE.write(line)
	info("Wrote a total %s lines to the target file." % len(NEW_FILE_LINES))
