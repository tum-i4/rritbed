#!/usr/bin/env python
""" TSP routing mapper """

from functionality.mapper_base import MapperBase

class TspRoutingMapper(MapperBase):
	""" Mapping position and target to some TSP route """

	GOAL_REACHED_MSG = "GOAL"

	@staticmethod
	def map(crd_x, crd_y, targ_x, targ_y):
		""" Maps the given position and target to some TSP route """
		if crd_x == targ_x and crd_y == targ_y:
			return TspRoutingMapper.GOAL_REACHED_MSG

		coordinates = TspRoutingMapper._get_coordinate_matrix()

		start = MapperBase._map(coordinates, crd_x, crd_y)
		end = MapperBase._map(coordinates, targ_x, targ_y)

		return "TSP Route from <{}> to <{}>".format(start, end)


	@staticmethod
	def _get_coordinate_matrix():
		coordinates = []

		for i in range(0, MapperBase._get_width()):
			coordinates.append([])

			for j in range(0, MapperBase._get_height()):
				coordinates[i].append("{},{}".format(i, j))

		MapperBase._assert_matrix_validity(coordinates)

		return coordinates
