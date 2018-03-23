#!/usr/bin/env python
""" Sequence utils """


def avg(sequence):
	""" Calculate the average value of a sequence. """
	return sum([float(x) for x in sequence]) / float(len(sequence))
