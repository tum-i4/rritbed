#!/usr/bin/env python
""" String formatters """


def format_time_passed(time_in_sec):
	""" Format the given seconds as "5 s", "2 m 10 s" or "1 h 2 m 1 s". """

	mins, secs = divmod(time_in_sec, 60)
	hours, mins = divmod(mins, 60)

	result = ""
	if hours > 0:
		result += "%d h " % hours
	if mins > 0:
		result += "%d m " % mins
	if secs > 0:
		result += "%d s" % secs

	return result.strip()


def format_percentage(value, pad_spaces=False):
	""" Formats the given float value in [0, 1] as a percentage string. """
	string = "%d %%" % round(value * 100, 2)
	if pad_spaces:
		return string.rjust(5)
	return string
