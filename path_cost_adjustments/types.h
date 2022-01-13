#pragma once
#include <vector>

//#define count 6
//#define depth1 10
//#define depth2 33
//#define depth3 56
//#define depth4 83
//#define depth5 156
//#define depth6 240

typedef unsigned int uint;

double getRandomDouble(int min, int max) {
	static const double fraction = 1.0 / (static_cast<double>(RAND_MAX) + 1.0);
	return static_cast<double>(rand() * fraction * (max - min + 1) + min);
}
int getRandomInt(int min, int max) {
	static const double fraction = 1.0 / (static_cast<int>(RAND_MAX) + 1.0);
	return static_cast<int>(rand() * fraction * (max - min + 1) + min);
}
double getVelosity(double d)
{
	double d_temp = d + getRandomDouble(-10, 10);
	d_temp /= 127.5;
	return pow(2.7,d_temp);
}

struct Point_
{
	double x, y;
	int clusterID;
};

class Point
{
public:
	uint id;
	uint x, y;
	double depth;
	double vel;
	std::vector<uint> connections;

	Point()
	{
	}
	Point(uint id_,uint x_,uint y_,uint depth_,std::vector<uint> connections_)
		:id(id_),x(x_),y(y_),depth(depth_),connections(connections_)
	{
	}

	Point_ getPoint_()
	{
		Point_ ret;
		ret.clusterID = -1;
		ret.x = depth;
		ret.y = vel;
		return ret;
	}
};

class RandomDepthGetter
{
	uint dispersion;
	std::vector<int> def_depth;
public:
	RandomDepthGetter(uint dispersion_, std::vector<int> def_depth_):dispersion(dispersion_),def_depth(def_depth_)
	{
	}
	double getRandomDepth()
	{
		double ret = getRandomDouble(0, static_cast<int>(dispersion));
		int r = getRandomInt(0, def_depth.size()-1);
		ret += def_depth[r];
		return ret;
	}
};



