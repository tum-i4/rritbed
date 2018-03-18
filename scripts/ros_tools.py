#!/usr/bin/env python
""" Easy-use interface for ROS control. """

import argparse
import os


BASE_PATH = os.path.expanduser("~/ros")
STOP_FILE_PATH = os.path.join(BASE_PATH, "STOP")


# pylint: disable-msg=W0613; (Unused argument)
def stop_call(args):
	""" Call _stop. Expects nothing. """
	_stop()


def _stop():
	""" Create a STOP file, wait for user input, then delete it again. """

	if os.path.lexists(STOP_FILE_PATH):
		print("STOP file exists already.")
	else:
		open(STOP_FILE_PATH, "a").close()
		print("STOP file created.")

	do_not = raw_input("Press [Enter] to delete the file again. Type 'do not' to not do that: ")

	if do_not == "do not":
		print("File NOT deleted.")
		return

	os.remove(STOP_FILE_PATH)
	print("File successfully deleted.")



if __name__ == "__main__":
	try:
		PARSER = argparse.ArgumentParser()
		SUBPARSERS = PARSER.add_subparsers()

		STOP_PARSER = SUBPARSERS.add_parser("stop", help="Create a STOP file to halt all running nodes.")
		STOP_PARSER.add_argument("please_do", choices=["please_do"], help="Ensures you want to do this")
		STOP_PARSER.set_defaults(function=stop_call)

		ARGS = PARSER.parse_args()

		ARGS.function(ARGS)
		exit()
	except KeyboardInterrupt:
		pass
