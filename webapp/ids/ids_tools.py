#!/usr/bin/env python
""" IDS tools """

import md5


def enumerate_to_dict(sequence, verify_hash):
	""" Enumerate the given sequence and save the items in a dict as item : index pairs. """

	mapping = {}
	for index, item in enumerate(sequence):
		mapping[item] = index

	verify_md5(mapping, verify_hash)
	return mapping


def verify_md5(obj, md5_hex_digest):
	""" Verify that the given object's string representation hashes to the given md5. """

	obj_hash = get_md5_hex(obj)
	if obj_hash != md5_hex_digest:
		raise ValueError("Invalid object given. Received: {}".format(obj_hash))


def get_md5_hex(obj):
	""" Create the md5 hex hash of the given object's string representation. """
	return md5.new(str(obj)).hexdigest()
