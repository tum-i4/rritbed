#!/usr/bin/env python
"""
VIN generator
For usage see --help output.
"""

import argparse
import random


class VinGenerator(object):
	""" Generates VINs """

	@staticmethod
	def run_with_args():
		""" Read arguments from command line and run accordingly """

		# VIN mode
		parser = argparse.ArgumentParser(prog="vgen", description="Generate VINs and output to stdout")

		parser.add_argument("vin_count", type=int, help="Number of VINs to generate")
		parser.add_argument("--dont-vary-plant", "-p", action="store_false", dest="vary_plant",
			help="Vary plant letter (default: yes)")

		args = parser.parse_args()

		VinGenerator._print_vins_and_exit(args.vin_count, args.vary_plant)


	@staticmethod
	def _print_vins_and_exit(number_of_vins, vary_plant=True):
		""" Generate VINs with the given arguments, print them one by one and exit """

		vins = VinGenerator.generate_vins(number_of_vins, vary_plant)
		for vin in vins:
			print(vin)

		exit()


	@staticmethod
	def generate_vins(number_of_vins, vary_plant=True):
		""" Generates VIN tails in the format [A-Z][0-9]{6} (from WBAUV710X0A192738) """

		possible_numbers = range(100000, 1000001-number_of_vins)
		selected_numbers = random.sample(possible_numbers, number_of_vins)

		vins = []
		plant_letter = VinGenerator._get_random_plant_letter()

		for serial_number in selected_numbers:
			if vary_plant:
				plant_letter = VinGenerator._get_random_plant_letter()
			vins.append(plant_letter + str(serial_number))

		return vins


	@staticmethod
	def _get_random_plant_letter():
		""" Returns char between A and Z """
		return chr(random.choice(range(65, 91)))



if __name__ == "__main__":
	VinGenerator.run_with_args()
