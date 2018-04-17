#!/usr/bin/env python
""" Statistics """

import statistics as stats


def avg(sequence):
	""" Short-hand for statistics.mean(). """
	return stats.mean(sequence)


def index_of_dispersion(sequence):
	""" Calculate the variance-to-mean ratio of a sequence. """
	float_seq = [float(x) for x in sequence]
	return stats.pvariance(float_seq) / stats.mean(float_seq)
