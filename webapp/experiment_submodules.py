#!/usr/bin/env python

# pylint: disable-msg=R0903; (Too few public methods)


class SubmoduleInterface(object):
	""" Interface for all submodules. """

	@staticmethod
	def run(experiment):
		""" abstract run() method """
		raise NotImplementedError


class AllVsSpecSvmVsIso(SubmoduleInterface):
	"""
	Experiment:
	Classifier trained on all vs specialised classifiers,
	OneClassSVM vs IsolationForest.
	"""

	@staticmethod
	def run(experiment):
		raise NotImplementedError()
