#!/usr/bin/env python
""" Launch file version check: Ensure all launch files are of the correct version. """

import argparse
import sys
import rospy

EXPECTED_VERSION = 1.2

# VERSIONS:
# 1.0 - Generators: Removes Zipf
# 1.1 - Generators: Removes --intrusion-mode zeroes, introduces --intrusion-mode off-value
#     - Generators: Adds --intrusion-level x
# 1.2 - Turtlesim:  Movement intelligence should always be 'return'

PARSER = argparse.ArgumentParser(prog="version_check",
	description="Ensures all launch files are of the correct version")
PARSER.add_argument("launch_file_version", metavar="X.Y", type=float,
	help="The version of the launch file to check.")
ARGS = PARSER.parse_args(rospy.myargv(sys.argv)[1:])

rospy.init_node("version_check", anonymous=True)

if ARGS.launch_file_version != EXPECTED_VERSION:
	INVALID_MSG = ("LAUNCH FILE VERSION {} INVALID! EXPECTING {}."
		.format(ARGS.launch_file_version, EXPECTED_VERSION))
	rospy.logfatal(INVALID_MSG)
else:
	rospy.loginfo("Launch file is up-to-date.")
