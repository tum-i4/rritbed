#!/usr/bin/env python
""" Analyse a log file """

import md5
import os
import statistics as stat

from log_entry import LogEntry
import util.fmtr
import util.outp
import util.prtr
from ids.dir_utils import Dir
import ids.ids_tools as ids_tools
import ids.ids_data as ids_data
import idse_dao


VERSION = "1.1"


def analyse(file_path, to_file, output_printer):
	""" Analyse the given log file. """

	# Check output file if requested #

	output_path = file_path + ".analysis"

	if to_file and os.path.lexists(output_path):
		raise IOError("Output file {} exists already! (Re)Move it and try again.".format(output_path))

	output_printer.prt("Analysing...")

	# Get file access #

	file_type = idse_dao.detect_type(file_path)
	if file_type == idse_dao.FileType.IDSE_FILE:
		print("Can't analyse IDSE files!")
		return
	elif file_type != idse_dao.FileType.LOG_FILE:
		raise NotImplementedError("File type \"%s\" not implemented!" % file_type)

	log_entry_generator = (LogEntry.from_log_string(line) for line in  Dir.yield_lines(file_path))

	# Analysis #

	all_app_ids = ids_data.get_app_ids()
	all_classes = ids_data.get_labels()

	(
		total_entries, found_app_ids, entry_count_per_app_id, elements_per_class_per_app_id,
		found_classes, entry_count_per_class, app_ids_per_class, duplicate_elements_per_app_id
	) = analyse_entries(log_entry_generator)

	# Output #

	printer = output_printer
	if to_file:
		printer = util.prtr.Storer()

	get_pl = lambda s, obj: s if len(obj) > 1 else ""

	if not to_file:
		printer.prt("")

	printer.prt("Analysis {}: Found {} entries with {}/{} app id{} and {}/{} class{}".format(
		VERSION, total_entries,
		len(found_app_ids), len(all_app_ids), get_pl("s", found_app_ids),
		len(found_classes), len(all_classes), get_pl("es", found_classes))
	)

	# "Elements and classes per app ID" table
	per_app_id = []
	per_app_id.append(["App ID", "Elements", "%"] + all_classes)
	for app_id in all_app_ids:
		line = [
			app_id,
			entry_count_per_app_id[app_id],
			util.fmtr.format_percentage(entry_count_per_app_id[app_id] / float(total_entries), True, 2)
		]

		for a_class in all_classes:
			class_count_str = ""
			if a_class in elements_per_class_per_app_id[app_id]:
				class_count_str = str(elements_per_class_per_app_id[app_id][a_class])
			line.append(class_count_str)

		per_app_id.append(line)

	util.outp.print_table(per_app_id, headline="Elements and classes per app ID", printer=printer)

	# "Elements per class" table
	per_class = []
	per_class.append(["Class", "Elements", "%", "App Ids"])
	for a_class in all_classes:
		per_class.append([
			a_class,
			entry_count_per_class[a_class],
			util.fmtr.format_percentage(entry_count_per_class[a_class] / float(total_entries), True, 2),
			len(app_ids_per_class[a_class])])

	util.outp.print_table(per_class, headline="Elements per class", printer=printer)

	# Duplicate table
	duplicates = []
	duplicates.append(["App ID", "All", "Unique", "Duplicates", "Duplicate %"])
	for app_id in all_app_ids:
		result = duplicate_elements_per_app_id[app_id]
		unique_count = result["uniq"]
		duplicate_count = result["dupe"]
		all_count = unique_count + duplicate_count
		duplicate_percent_str = util.fmtr.format_percentage(0)
		if all_count > 0:
			duplicate_percent_str = util.fmtr.format_percentage(
				float(duplicate_count) / all_count, True, 3)

		duplicates.append([app_id, all_count, unique_count, duplicate_count, duplicate_percent_str])

	# Check content (skip header) for found duplicates
	if not any([l[3] > 0 for l in duplicates[1:]]):
		printer.prt("\nDuplicate analysis: No duplicates found!")
	else:
		util.outp.print_table(duplicates, headline="Duplicates per app ID", printer=printer)

	if to_file:
		with open(output_path, "w") as output_file:
			for line in printer.get_messages():
				output_file.write(line + "\n")

		output_printer.prt("Successfully saved analysis to \"{}\".".format(output_path))

	# TODO: score??
	# harmonious? all labelled / some / none?
	# for each app id: are there roughly the same number of entries per class?
	return


def analyse_entries(log_entry_generator):
	"""
	Analyse the LogEntry objects from the given generator.
	returns: A tuple containing (found_app_ids, entry_count_per_app_id, elements_per_class_per_app_id,
	found_classes, entry_count_per_class, app_ids_per_class, duplicate_elements_per_app_id)
	"""

	total_entries = 0

	all_app_ids = ids_data.get_app_ids()
	found_app_ids = set()
	entry_count_per_app_id = {}
	elements_per_class_per_app_id = {}

	all_classes = ids_data.get_labels()
	found_classes = set()
	entry_count_per_class = {}
	app_ids_per_class = {}

	duplicate_elements_per_app_id = {}
	last_hash_per_app_id = {}

	for app_id in all_app_ids:
		entry_count_per_app_id[app_id] = 0
		elements_per_class_per_app_id[app_id] = {}

		# Unique, Duplicates
		duplicate_elements_per_app_id[app_id] = dict(uniq=0, dupe=0)
		last_hash_per_app_id[app_id] = None

	for a_class in all_classes:
		entry_count_per_class[a_class] = 0
		app_ids_per_class[a_class] = set()

	for entry in log_entry_generator:
		if not entry.intrusion:
			raise NotImplementedError("Entries without labels can currently not be handled")

		total_entries += 1

		app_id = ids_tools.log_entry_to_app_id(entry)
		its_class = entry.intrusion

		found_app_ids.add(app_id)
		found_classes.add(its_class)

		entry_count_per_app_id[app_id] += 1

		if its_class not in elements_per_class_per_app_id[app_id]:
			elements_per_class_per_app_id[app_id][its_class] = 1
		else:
			elements_per_class_per_app_id[app_id][its_class] += 1

		entry_count_per_class[its_class] += 1
		app_ids_per_class[its_class].add(app_id)

		entry_hash = get_content_hash(entry)
		if entry_hash == last_hash_per_app_id[app_id]:
			duplicate_elements_per_app_id[app_id]["dupe"] += 1
		else:
			duplicate_elements_per_app_id[app_id]["uniq"] += 1
		last_hash_per_app_id[app_id] = entry_hash


	return (total_entries, found_app_ids, entry_count_per_app_id, elements_per_class_per_app_id,
		found_classes, entry_count_per_class, app_ids_per_class, duplicate_elements_per_app_id)


def get_content_hash(log_entry):
	""" Hash only the content of the given log entry.
	Omits app_id, time_unix and log id. Includes label. """

	entry_string = log_entry.data[LogEntry.VIN_FIELD]
	entry_string += log_entry.data[LogEntry.LEVEL_FIELD]
	entry_string += log_entry.data[LogEntry.GPS_POSITION_FIELD]
	entry_string += log_entry.data[LogEntry.LOG_MESSAGE_FIELD]
	entry_string += log_entry.intrusion

	return md5.new(entry_string).hexdigest()
