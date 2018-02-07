#!/usr/bin/env python
""" TSP routing mapper """

from functionality.mapper_base import MapperBase

class TspRoutingMapper(MapperBase):
	""" Mapping position and target to some TSP route """

	@staticmethod
	def map(crd_x, crd_y, targ_x, targ_y):
		""" Maps the given position and target to some TSP route """
		pass


	@staticmethod
	def _get_coordinate_matrix():
		coordinates = [
			[],
			[],
			[],
			[],
			[]
		]

		for i in range(0, MapperBase._get_width()):
			for j in range(0, MapperBase._get_height()):
				coordinates[i][j] = "{},{}".format(i, j)

		MapperBase._assert_matrix_validity(coordinates)

		return coordinates
