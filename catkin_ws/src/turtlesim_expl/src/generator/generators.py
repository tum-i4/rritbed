#!/usr/bin/env python
""" All generator definitions and corresponding methods """

import argparse
import json
import os

import numpy as np
from argument_constraint import ArgumentConstraint as AC
from distribution_generator import DistributionGenerator


GAUSSIAN_STR = "gaussian"
GUMBEL_STR = "gumbel"
LAPLACE_STR = "laplace"
LOGISTIC_STR = "logistic"
PARETO_STR = "pareto"
RAYLEIGH_STR = "rayleigh"
UNIFORM_STR = "uniform"
VON_MISES_STR = "vonmises"
WALD_STR = "wald"
WEIBULL_STR = "weibull"
ZIPF_STR = "zipf"

EASY_STR = "easy"
MED_STR = "med"
HARD_STR = "hard"


# off_value_values: EASY +/- 10 *, MED +/- 5 *, HARD: +/- 1.5 *


# pylint: disable-msg=E1101
GENERATORS = {
	# Gaussian, Gumbel, Laplace: loc and scale arbitrary
	GAUSSIAN_STR : DistributionGenerator("normal", GAUSSIAN_STR,
		[AC(0.0), AC(1.0)],
		expected_values=[-3.5, 3.5],
		huge_error_lambdas={}),
	# GUMBEL_STR : DistributionGenerator("gumbel", GUMBEL_STR,
	# 	[AC(0.0), AC(1.0)],
	# 	# Expects -7 ~ 2.3
	# 	off_value_values={EASY_STR:[-70, 23], MED_STR:[-35, 11.5], HARD_STR:[-10.5, 3.45]},
	# 	huge_error_lambdas={}),
	# LAPLACE_STR : DistributionGenerator("laplace", LAPLACE_STR,
	# 	[AC(0.0), AC(1.0)],
	# 	# Expects -6.3 ~ 6.3
	# 	off_value_values={EASY_STR:[-63, 63], MED_STR:[-31.5, 31.5], HARD_STR:[-9.45, 9.45]},
	# 	huge_error_lambdas={}),
	# # Logistic: loc arbitrary, scale > 0
	# LOGISTIC_STR : DistributionGenerator("logistic", LOGISTIC_STR,
	# 	[AC(0.0), AC(1.0, min_value=0.1)],
	# 	# Expects -TODO ~ TODO
	# 	off_value_values={EASY_STR:[-35, 35], MED_STR:[-17.5, 17.5], HARD_STR:[-5.25, 5.25]},
	# 	huge_error_lambdas={}),
	# # Pareto: a(lpha) > 0
	# PARETO_STR : DistributionGenerator("pareto", PARETO_STR,
	# 	[AC(1.0, min_value=0.1)],
	# 	# Expects -TODO ~ TODO
	# 	off_value_values={EASY_STR:[-35, 35], MED_STR:[-17.5, 17.5], HARD_STR:[-5.25, 5.25]},
	# 	huge_error_lambdas={}),
	# # Rayleigh: scale > 0
	# RAYLEIGH_STR : DistributionGenerator("rayleigh", RAYLEIGH_STR,
	# 	[AC(1.0, min_value=0.1)],
	# 	# Expects -TODO ~ TODO
	# 	off_value_values={EASY_STR:[-35, 35], MED_STR:[-17.5, 17.5], HARD_STR:[-5.25, 5.25]},
	# 	huge_error_lambdas={}),
	# # Uniform: low < high (not a binding constraint)
	# UNIFORM_STR : DistributionGenerator("uniform", UNIFORM_STR,
	# 	[AC(0.0), AC(1.0)],
	# 	# Expects -TODO ~ TODO
	# 	off_value_values={EASY_STR:[-35, 35], MED_STR:[-17.5, 17.5], HARD_STR:[-5.25, 5.25]},
	# 	huge_error_lambdas={}),
	# # Von Mises: mu arbitrary, kappa >= 0
	# VON_MISES_STR : DistributionGenerator("vonmises", VON_MISES_STR,
	# 	[AC(0.0), AC(1.0, min_value=0)],
	# 	# Expects -TODO ~ TODO
	# 	off_value_values={EASY_STR:[-35, 35], MED_STR:[-17.5, 17.5], HARD_STR:[-5.25, 5.25]},
	# 	huge_error_lambdas={}),
	# # Wald: mean > 0, scale > 0
	# WALD_STR : DistributionGenerator("wald", WALD_STR,
	# 	[AC(1.0, min_value=0.1), AC(1.0, min_value=0.1)],
	# 	# Expects -TODO ~ TODO
	# 	off_value_values={EASY_STR:[-35, 35], MED_STR:[-17.5, 17.5], HARD_STR:[-5.25, 5.25]},
	# 	huge_error_lambdas={}),
	# # Weibull: a > 0
	# WEIBULL_STR : DistributionGenerator("weibull", WEIBULL_STR,
	# 	[AC(5.0, min_value=0.1)],
	# 	# Expects -TODO ~ TODO
	# 	off_value_values={EASY_STR:[-35, 35], MED_STR:[-17.5, 17.5], HARD_STR:[-5.25, 5.25]},
	# 	huge_error_lambdas={}),
	# # Zipf: a > 1
	# ZIPF_STR : DistributionGenerator("zipf", ZIPF_STR,
	# 	[AC(2.0, min_value=1.1)],
	# 	# Expects -TODO ~ TODO
	# 	off_value_values={EASY_STR:[-35, 35], MED_STR:[-17.5, 17.5], HARD_STR:[-5.25, 5.25]},
	# 	huge_error_lambdas={})
}


def get_generator_names():
	"""	Return a list of names of all currently available generators
	(used as keys for the generator definition dictionary).	"""
	return GENERATORS.keys()


def _generator_mode(file_path):
	""" Print all current generator definitions out (file_path is None) or write them to a file """

	if file_path is not None:
		_generator_mode_write_to_file(file_path)
		exit()

	print("name, ['default, min, max', ...]")
	for gen_name in GENERATORS:
		constraints = GENERATORS[gen_name].args_constraints
		print("{}, {}".format(gen_name, ["{}, {}, {}".format(
			x.default_value, x.min_value, x.max_value)
			for x in constraints]))


def _generator_mode_write_to_file(file_path):
	file_path = os.path.expanduser(file_path)

	folder_path = os.path.dirname(file_path)
	if not os.path.exists(folder_path):
		print("File path {} does not exist".format(folder_path))
		exit()

	if os.path.exists(file_path):
		print("File {} exists and would be overwritten".format(file_path))
		exit()

	gen_defs = {}

	for gen_name in GENERATORS:
		constraints = GENERATORS[gen_name].args_constraints
		gen_defs[gen_name] = [
			{"default" : x.default_value, "min" : x.min_value, "max" : x.max_value}
			for x in constraints]

	result = json.dumps(gen_defs)

	with open(file_path, 'w') as file_writer:
		file_writer.write(result)

	print("Data written to file {} successfully".format(file_path))


if __name__ == "__main__":
	PARSER = argparse.ArgumentParser()
	PARSER.add_argument("file_path", nargs="?", metavar="/FILE/PATH", default=None,
		help="File path to store the definitions to")
	ARGS = PARSER.parse_args()

	_generator_mode(ARGS.file_path)
