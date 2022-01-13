#pragma once
#include "types.h"

class RouteGenerator
{
	uint size_map;
	uint dispersion;
public:
	RouteGenerator(uint size_map_,uint dispersion_):size_map(size_map_), dispersion(dispersion_)
	{}
	std::vector<Point> getNewMap()
	{
		std::vector<std::vector<Point>> map;
		std::vector<Point> ret;
		map.resize(size_map);
		ret.reserve(size_map*size_map);
		for (int i = 0; i < map.size(); i++)
			map[i].resize(size_map);
		uint id = 1;
		for (int i = 0; i < map.size(); ++i)
		{
			for (int j = 0; j < map.size(); j++)
			{
				map[i][j].id = id;
				map[i][j].y = i;
				map[i][j].x = j;
				id++;
				RandomDepthGetter g(dispersion, {10,41,71,133,170,233});
				map[i][j].depth = g.getRandomDepth();
				map[i][j].vel = getVelosity(map[i][j].depth);
			}
		}
		for (int i = 0; i < map.size(); ++i)
		{
			for (int j = 0; j < map.size(); j++)
			{
				if(j!=0)
					map[i][j].connections.push_back(map[i][j-1].id);
				if (j+1 < size_map)
					map[i][j].connections.push_back(map[i][j + 1].id);
				if (i != 0)
					map[i][j].connections.push_back(map[i-1][j].id);
				if (i+1 < size_map)
					map[i][j].connections.push_back(map[i + 1][j].id);
			}
		}
		for (int i = 0; i < map.size(); ++i)
			for (int j = 0; j < map.size(); j++)
				ret.push_back(map[i][j]);

		return ret;
	}
};