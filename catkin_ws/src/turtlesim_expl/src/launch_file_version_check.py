#!/usr/bin/env python
""" Launch file version check: Ensure all launch files are of the correct version. """

import argparse
import rospy

EXPECTED_VERSION = 1.1

PARSER = argparse.ArgumentParser(prog="version_check",
	description="Ensures all launch files are of the correct version")
PARSER.add_argument("launch_file_version", metavar="X.Y",
	help="The version of the launch file to check.")
ARGS = PARSER.parse_args()

if ARGS.launch_file_version != EXPECTED_VERSION:
	INVALID_MSG = "LAUNCH FILE VERSION INVALID!"
	rospy.logfatal(INVALID_MSG)
	rospy.signal_shutdown(INVALID_MSG)
	raise rospy.ROSException(INVALID_MSG)
