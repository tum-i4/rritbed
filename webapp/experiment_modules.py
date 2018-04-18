#!/usr/bin/env python
""" Runnable experiments. All respect the ModuleInterface interface. """

# pylint: disable-msg=R0903; (Too few public methods)


class ModuleInterface(object):
	""" Interface for all submodules. """

	@staticmethod
	def run(experiment):
		""" abstract run() method """
		raise NotImplementedError


class AllVsSpecSvmVsIso(ModuleInterface):
	"""
	Experiment:
	Classifier trained on all vs specialised classifiers,
	OneClassSVM vs IsolationForest.
	"""

	@staticmethod
	def run(experiment):
		raise NotImplementedError()
