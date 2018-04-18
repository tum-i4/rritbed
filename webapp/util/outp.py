#!/usr/bin/env python
""" Outputters """

import fmtr
import prtr


def print_table(list_of_lists, headline=None, col_sep=" | ", head_sep=True, printer=None):
	""" Print the given list of tuple as a table, regarding the first entry the header. """

	if len(list_of_lists) < 2:
		raise ValueError("Input list needs at least one header and one content line!")

	if printer is None:
		printer = prtr.Printer()

	table_column_count = len(list_of_lists[0])
	max_width_per_column = [0] * table_column_count

	for one_list in list_of_lists:
		if len(one_list) != table_column_count:
			raise ValueError("One or more of the given input lines have different numbers of entries!")

		for i, one_el in enumerate(one_list):
			max_width_per_column[i] = max(max_width_per_column[i], len(str(one_el)))

	lines_to_print = []
	for one_list in list_of_lists:
		justed_strings = []
		for i, one_el in enumerate(one_list):
			justed_strings.append((str(one_el).ljust(max_width_per_column[i])))
		lines_to_print.append(col_sep.join(justed_strings))

	# Each column plus col_sep length (" | ")
	table_width = sum(max_width_per_column) + len(col_sep) * (table_column_count - 1)

	# Print #

	printer.prt("")
	if headline:
		if not isinstance(headline, str):
			raise ValueError("headline must be string")

		printer.prt(headline.center(table_width))

	printer.prt(lines_to_print[0])

	if head_sep:
		printer.prt("-" * table_width)

	for i in range(1, len(lines_to_print)):
		printer.prt(lines_to_print[i])


### User info ###


def print_time_passed_message(time_passed, task_name=None):
	""" Print a message about how long a task took. """

	message = "Finished "
	if task_name is not None:
		message += "task '%s'" % task_name
	message += "in %s" % fmtr.format_time_passed(time_passed)
	print(message)


### Print shortcuts ###


def exit_on_error(error_message):
	""" Print the given error message and call exit(). """

	prtr.Printer(name="Error").prt(error_message)
	exit()


def print_debug(debug_message):
	""" Print the given message as a debug output. """

	prtr.Printer(name="Debug").prt(debug_message)
