// path_cost_adjustments.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include <iostream>
#include <ctime>
#include <functional>
#include <set>
#include <fstream>
#include "types.h"
#include "RouteGenerator.h"
#include "path_planning.h"
#include "dbscan.h"

const int data_size = 30;
const int map_size = 10;
const int dispersion_value = 15;
int main()
{
	srand((unsigned int)time(NULL));
	
	std::vector<std::vector<Point>> map_data(data_size);
	std::vector<std::pair<int, int>> target_data(data_size);
	std::vector<Point> path_data1;
	path_data1.reserve(map_size*map_size*5);
	double time1 = 0;

	std::vector<Point> path_data2;
	path_data1.reserve(map_size*map_size * 5);
	double time2 = 0;

	std::function<double(Point, Point)> f = [](Point a, Point b) {return sqrt(pow((int)a.x - (int)b.x, 2) + pow((int)a.y - (int)b.y, 2)); };
	RouteGenerator generator(map_size, dispersion_value);
	for (int i = 0; i < map_data.size(); i++)
	{
		map_data[i] = generator.getNewMap();
		RouteKeeper keeper(map_data[i], f);
		target_data[i].first = 0; target_data[i].second = 0;
		while (target_data[i].first == target_data[i].second)
		{
			target_data[i].first = getRandomInt(1, map_size*map_size);
			target_data[i].second = getRandomInt(1, map_size*map_size);
		}
		auto output = keeper.get_path(target_data[i].first, target_data[i].second);
		double time = 0;
		for (int i = 1; i < output.size(); ++i)
		{
			auto c = f(output[i - 1], output[i]);
			time += output[i].vel*c;
		}
		time /= output.size();
		time1 += time;
		for (auto it = output.begin(); it != output.end(); it++)
			path_data1.push_back(*it);
	}
	time1 /= map_data.size();

	std::vector<Point_> dbs_data(path_data1.size());
	for (int i = 0; i < dbs_data.size(); ++i)
		dbs_data[i] = path_data1[i].getPoint_();

	DBSCAN<Point_> d(9,16,dbs_data);
	d.run();


	std::cout << d.getTotalPointSize()<<std::endl;
	std::set<int> clusters;


	for (auto i : d.m_points)
		clusters.insert(i.clusterID);
	std::cout << clusters.size() << std::endl;

	std::function<double(Point, Point)> f2 = [&d](Point a, Point b) {auto out = d.calculateClusterSimple(b.getPoint_()); return out* sqrt(pow((int)a.x - (int)b.x, 2) + pow((int)a.y - (int)b.y, 2)); };
	for (int i = 0; i < map_data.size(); i++)
	{
		RouteKeeper keeper(map_data[i], f2);
		auto output = keeper.get_path(target_data[i].first, target_data[i].second);
		double time = 0;
		for (int i = 1; i < output.size(); ++i)
		{
			time += output[i].vel*f(output[i - 1], output[i]);
		}
		time /= output.size();
		time2 += time;
		for (auto it = output.begin(); it != output.end(); it++)
			path_data2.push_back(*it);
	}
	time2 /= map_data.size();

	std::cout << "Average path search time: "<<time1 << std::endl;
	std::cout << "Average pathfinding time using DBSCAN: " << time2 << std::endl;
	std::cout << "Reducing search time by: " << 100*(1 - (time2/time1))<<"%" << std::endl;
	std::cout << "Number of clusters: "<<clusters.size() << std::endl;

	std::ofstream out;          
	out.open("output.txt");
	if (out.is_open())
	{
		for (auto it = clusters.begin(); it != clusters.end(); it++)
		{
			for (auto i : d.m_points)
			{
				if (*it == i.clusterID)
				{
					out <<i.clusterID<<"\t"<<i.x<<"\t" <<i.y << std::endl;
				}
			}
			out << std::endl;
			out << "===========================";
			out << std::endl;
		}
		
	}
}

