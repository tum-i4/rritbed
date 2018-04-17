#!/usr/bin/env python
""" Sequence utils """


def yield_items_in_key_order(any_dict):
	""" Replacement for items() that sorts keys before returning. """

	sorted_keys = sorted(any_dict.keys())
	for key in sorted_keys:
		yield (key, any_dict[key])
