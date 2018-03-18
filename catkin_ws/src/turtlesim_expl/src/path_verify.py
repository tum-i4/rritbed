#!/usr/bin/env python
""" verify() method """

import os

import generator.distribution_publisher as DP
import mover.basic_mover as BM
import mover.turtle_control as TC

BASE_PATH = os.path.expanduser("~/ros")

def verify():
	""" Verify that the classes distribution_publisher.py and basic_mover.py\
	use the correct BASE_PATH. """

	result = []
	for module in [DP, BM, TC]:
		msg = "{}:".format(os.path.splitext(os.path.basename(module.__file__))[0])
		if module.BASE_PATH != BASE_PATH:
			print("{} invalid [{}]".format(msg, module.BASE_PATH))
			result.append(False)
		else:
			print("{} okay".format(msg))
			result.append(True)

	result_msg = "{}/{} tests successful".format(result.count(True), len(result))
	print("-" * len(result_msg))
	print(result_msg)



if __name__ == "__main__":
	verify()
