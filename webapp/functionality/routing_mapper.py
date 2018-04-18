#!/usr/bin/env python
""" TSP routing mapper """

from functionality.mapper_base import MapperBase

class RoutingMapper(MapperBase):
	""" Mapping position and target to some TSP route """

	@staticmethod
	def map(crd_x, crd_y, targ_x, targ_y):
		""" Convert the given coordinates to "x,y,t_x,t_y". """
		return "{},{},{},{}".format(crd_x, crd_y, targ_x, targ_y)


	@staticmethod
	def _get_coordinate_matrix():
		coordinates = []

		for i in range(0, MapperBase._get_width()):
			coordinates.append([])

			for j in range(0, MapperBase._get_height()):
				coordinates[i].append("{},{}".format(i, j))

		MapperBase._assert_matrix_validity(coordinates)

		return coordinates
