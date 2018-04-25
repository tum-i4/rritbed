#!/usr/bin/env python
""" Analyse a log file """

import md5
import os

from log_entry import LogEntry
import util.fmtr
import util.outp
import util.prtr
import util.stat
from ids.dir_utils import Dir
import ids.ids_tools as ids_tools
import ids.ids_data as ids_data
import idse_dao


VERSION = "1.2"


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
		found_classes, entry_count_per_class, app_ids_per_class, duplicate_elements_per_app_id,
		scorable_app_ids, dispersion_index, duplicate_index
	) = analyse_entries(log_entry_generator)

	# Output #

	printer = output_printer
	if to_file:
		printer = util.prtr.Storer()

	get_pl = lambda s, obj: s if len(obj) > 1 else ""
	total_line_name = "<total>"

	if not to_file:
		printer.prt("")

	printer.prt("Analysis {}: Found {:,} entries with {}/{} app id{} and {}/{} class{}".format(
		VERSION, total_entries,
		len(found_app_ids), len(all_app_ids), get_pl("s", found_app_ids),
		len(found_classes), len(all_classes), get_pl("es", found_classes))
	)

	# "Elements and classes per app ID" table
	per_app_id = []
	per_app_id.append(["App ID", "Elements", "El. %"] + all_classes)
	total_entries_assertion = 0
	for app_id in all_app_ids:
		total_entries_assertion += entry_count_per_app_id[app_id]

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

	assert(total_entries == total_entries_assertion)

	empty_line = [""] * (3 + len(all_classes))
	per_app_id.append(empty_line)

	total_line = [
		total_line_name,
		"{:,}".format(total_entries),
		util.fmtr.format_percentage(1, True, 2)
	]
	for a_class in all_classes:
		total_line.append("{:,}".format(entry_count_per_class[a_class]))
	per_app_id.append(total_line)

	util.outp.print_table(per_app_id, headline="Elements and classes per app ID", printer=printer)

	# "per class" table
	per_class = []
	per_class.append([""] + all_classes)
	app_ids_line = ["App IDs"]
	percent_line = ["Percentage"]
	for a_class in all_classes:
		app_ids_line.append(
			len(app_ids_per_class[a_class]))
		percent_line.append(
			util.fmtr.format_percentage(entry_count_per_class[a_class] / float(total_entries), False, 2))

	per_class.append(app_ids_line)
	per_class.append(percent_line)

	util.outp.print_table(per_class, headline="Metrics per class", printer=printer)

	# Duplicate table
	duplicates = []
	duplicates.append(["App ID", "All", "Unique", "Duplicates", "Duplicate %"])
	total_number_of_duplicates = 0
	total_entries_assertion = 0
	for app_id in all_app_ids:
		result = duplicate_elements_per_app_id[app_id]
		unique_count = result["uniq"]
		duplicate_count = result["dupe"]
		all_count = unique_count + duplicate_count

		total_number_of_duplicates += duplicate_count
		total_entries_assertion += all_count

		duplicate_percent = 0
		if all_count > 0:
			duplicate_percent = float(duplicate_count) / all_count
		duplicate_percent_str = util.fmtr.format_percentage(duplicate_percent, True, 3)

		duplicates.append([app_id, all_count, unique_count, duplicate_count, duplicate_percent_str])

	assert(total_entries == total_entries_assertion)

	# Don't output table if there are no duplicates
	if total_number_of_duplicates == 0:
		printer.prt("\nDuplicate analysis: No duplicates found!")
	else:
		empty_line = [""] * 5
		duplicates.append(empty_line)

		total_duplicate_percent = float(total_number_of_duplicates) / total_entries
		duplicates.append([
			total_line_name,
			"{:,}".format(total_entries),
			"{:,}".format(total_entries - total_number_of_duplicates),
			"{:,}".format(total_number_of_duplicates),
			util.fmtr.format_percentage(total_duplicate_percent, True, 3)
		])
		util.outp.print_table(duplicates, headline="Duplicates per app ID", printer=printer)

	printer.prt("\nScores for %s scorable app ids: Dispersion index = %s | Duplicate index = %s"
		% (len(scorable_app_ids), round(dispersion_index, 3), round(duplicate_index, 3))
	)
	printer.prt("Scorable app ids: %s" % scorable_app_ids)

	if to_file:
		with open(output_path, "w") as output_file:
			for line in printer.get_messages():
				output_file.write(line + "\n")

		output_printer.prt("Successfully saved analysis to \"{}\".".format(output_path))

	# harmonious? all labelled / some / none?
	# for each app id: are there roughly the same number of entries per class?
	return


def analyse_entries(log_entry_generator):
	"""
	Analyse the LogEntry objects from the given generator.
	returns: A tuple containing (found_app_ids, entry_count_per_app_id, elements_per_class_per_app_id,
	found_classes, entry_count_per_class, app_ids_per_class, duplicate_elements_per_app_id),
	scorable_app_ids, dispersion_index, duplicate_index
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

	scorable_app_ids = []
	scorable_entry_counts = []
	scorable_duplicate_percentages = []

	for app_id in all_app_ids:
		entry_count = entry_count_per_app_id[app_id]
		dupe_count = duplicate_elements_per_app_id[app_id]["dupe"]
		if entry_count > 0:
			scorable_app_ids.append(app_id)
			scorable_entry_counts.append(entry_count)
			scorable_duplicate_percentages.append(float(dupe_count)/entry_count)

	dispersion_index = util.stat.index_of_dispersion(scorable_entry_counts)
	duplicate_index = (util.stat.avg(scorable_duplicate_percentages)
		+ max(scorable_duplicate_percentages) / float(len(scorable_duplicate_percentages)))

	return (total_entries, found_app_ids, entry_count_per_app_id, elements_per_class_per_app_id,
		found_classes, entry_count_per_class, app_ids_per_class, duplicate_elements_per_app_id,
		scorable_app_ids, dispersion_index, duplicate_index)


def get_content_hash(log_entry):
	""" Hash only the content of the given log entry.
	Omits app_id, time_unix and log id. Includes label. """

	entry_string = log_entry.data[LogEntry.VIN_FIELD]
	entry_string += log_entry.data[LogEntry.LEVEL_FIELD]
	entry_string += log_entry.data[LogEntry.GPS_POSITION_FIELD]
	entry_string += log_entry.data[LogEntry.LOG_MESSAGE_FIELD]
	entry_string += log_entry.intrusion

	return md5.new(entry_string).hexdigest()
