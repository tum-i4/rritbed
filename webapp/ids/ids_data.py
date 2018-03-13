#!/usr/bin/env python
""" Shared data among IDS components """

# pylint: disable-msg=C0111; (Missing method docstring)

from functionality.poi_mapper import PoiMapper as PoMa
from log_entry import LogEntry

# App IDs
POSE_CC = "COUNTRYCODE"
POSE_POI = "POI"
POSE_TSP = "TSPROUTING"
def get_generators():
	return list(["GAUSSIAN", "GUMBEL", "LAPLACE", "LOGISTIC", "PARETO", "RAYLEIGH",
	"UNIFORM", "VONMISES", "WALD", "WEIBULL", "ZIPF"])
def get_colours():
	return list(["COLOUR"])
def get_poses():
	return list([POSE_CC, POSE_POI, POSE_TSP])
def get_app_ids():
	return list(get_generators() + get_colours() + get_poses())

# Levels
def get_levels():
	return list([LogEntry.LEVEL_DEFAULT, LogEntry.LEVEL_ERROR])

# POI types
def get_legal_poi_types():
	return list([PoMa.restaurants_field, PoMa.gas_stations_field])
def get_intruded_poi_types():
	return list(["private home", "nsa hq"])
def get_poi_types():
	return list(get_legal_poi_types() + get_intruded_poi_types())
# POI results
def get_legal_poi_results():
	return list([PoMa.ita, PoMa.ger, PoMa.frc, PoMa.tot, PoMa.shl, PoMa.arl])
def get_intruded_poi_results():
	return list(["Invalid"])
def get_poi_results():
	return list(get_legal_poi_results() + get_intruded_poi_results())

# Labels
def get_legal_labels():
	return list(["normal"])
def get_intrusion_labels_gens():
	return list(["zeroes", "huge-error"])
def get_labels_gens():
	return list(get_legal_labels() + get_intrusion_labels_gens())
def get_intrusion_labels_colrs():
	return list(["red"])
def get_labels_colrs():
	return list(get_legal_labels() + get_intrusion_labels_colrs())
def get_intrusion_labels_pose_cc():
	return list(["jump"])
def get_labels_pose_cc():
	return list(get_legal_labels() + get_intrusion_labels_pose_cc())
def get_intrusion_labels_pose_poi():
	return list(["illegaltype"])
def get_labels_pose_poi():
	return list(get_legal_labels() + get_intrusion_labels_pose_poi())
def get_intrusion_labels_pose_tsp():
	return list(["routetoself"])
def get_labels_pose_tsp():
	return list(get_legal_labels() + get_intrusion_labels_pose_tsp())
def get_intrusion_labels_poses():
	return list(get_intrusion_labels_pose_cc() + get_intrusion_labels_pose_poi()
		+ get_intrusion_labels_pose_tsp())
def get_labels_poses():
	return list(get_legal_labels() + get_intrusion_labels_poses())
def get_intrusion_labels():
	return (
		list(get_intrusion_labels_gens() + get_intrusion_labels_colrs() + get_intrusion_labels_poses()))
def get_labels():
	return list(get_legal_labels() + get_intrusion_labels())

# MD5 hex hashes
APP_IDS_MD5 = "cacafa61f61b645c279954952ac6ba8f"
LEVEL_MAPPING_MD5 = "49942f0268aa668e146e533b676f03d0"
POI_TYPE_MAPPING_MD5 = "f2fba0ed17e382e274f53bbcb142565b"
POI_RESULT_MAPPING_MD5 = "dd1c18c7188a48a686619fef8007fc64"
LABEL_INT_MAPPING_MD5 = "69a262192b246d16e8411b6db06e237b"
INT_LABEL_MAPPING_MD5 = "c29a85dae460b57fac78db12e72ae24a"
