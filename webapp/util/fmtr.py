#!/usr/bin/env python
""" String formatters """


def format_time_passed(time_in_sec):
	""" Format the given seconds as "5 s", "2 m 10 s" or "1 h 2 m 1 s". """

	if time_in_sec < 0:
		raise ValueError("Time passed can only be positive!")

	mins, secs = divmod(time_in_sec, 60)
	hours, mins = divmod(mins, 60)

	result = ""
	if hours > 0:
		result += "%d h " % hours
	if mins > 0:
		result += "%d m " % mins
	if secs > 0 or time_in_sec == 0:
		result += "%d s" % secs

	return result.strip()


def format_percentage(value, pad_spaces=False, digits=0):
	""" Formats the given float value in [0, 1] as a percentage string. """

	# For no digits: 'xxx %'
	just_len = 5
	if digits > 0:
		# Normal + '.dd'
		just_len += digits + 1
	elif digits < 0:
		raise ValueError("digits must be a value >= 0")

	formatter = "{:.%sf} %%" % digits

	string = formatter.format(value * 100)
	if pad_spaces:
		return string.rjust(just_len)
	return string
