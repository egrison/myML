#pragma once

#include <vector>
#include <cmath>


#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define SUCCESS 0
#define FAILURE -3

template <typename PointD>
class DBSCAN {
public:
	DBSCAN(unsigned int minPts,double eps, std::vector<PointD> points) {
		m_minPoints = minPts;
		m_epsilon = eps;
		m_points = points;
		m_pointSize = points.size();
	}
	~DBSCAN() {}

	int run()
	{
		int clusterID = 1;
		//std::vector<PointD>::iterator iter;
		for (auto iter = m_points.begin(); iter != m_points.end(); ++iter)
		{
			if (iter->clusterID == UNCLASSIFIED)
			{
				if (expandCluster(*iter, clusterID) != FAILURE)
				{
					clusterID += 1;
				}
			}
		}

		return 0;
	}

	std::vector<int> calculateCluster(PointD point)
	{
		int index = 0;
		//std::vector<PointD>::iterator iter;
		std::vector<int> clusterIndex;
		for (auto iter = m_points.begin(); iter != m_points.end(); ++iter)
		{
			if (calculateDistance(point, *iter) <= m_epsilon)
			{
				clusterIndex.push_back(index);
			}
			index++;
		}
		return clusterIndex;
	}
	double calculateClusterSimple(PointD point)
	{
		int index = 0;
		//std::vector<PointD>::iterator iter;
		std::vector<int> clusterIndex;
		for (auto iter = m_points.begin(); iter != m_points.end(); ++iter)
		{
			if (calculateDistance(point, *iter) <= m_epsilon)
				clusterIndex.push_back(index);
			index++;
		}
		double out = 0;
		for (int i = 0; i < clusterIndex.size(); ++i)
		{
			out += m_points[clusterIndex[i]].y;
		}
		return out/clusterIndex.size();
	}

	int expandCluster(PointD point, int clusterID)
	{
		std::vector<int> clusterSeeds = calculateCluster(point);

		if (clusterSeeds.size() < m_minPoints)
		{
			point.clusterID = NOISE;
			return FAILURE;
		}
		else
		{
			int index = 0, indexCorePoint = 0;
			std::vector<int>::iterator iterSeeds;
			for (iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
			{
				m_points.at(*iterSeeds).clusterID = clusterID;
				if (m_points.at(*iterSeeds).x == point.x && m_points.at(*iterSeeds).y == point.y)
				{
					indexCorePoint = index;
				}
				++index;
			}
			clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

			for (std::vector<int>::size_type i = 0, n = clusterSeeds.size(); i < n; ++i)
			{
				std::vector<int> clusterNeighors = calculateCluster(m_points.at(clusterSeeds[i]));

				if (clusterNeighors.size() >= m_minPoints)
				{
					std::vector<int>::iterator iterNeighors;
					for (iterNeighors = clusterNeighors.begin(); iterNeighors != clusterNeighors.end(); ++iterNeighors)
					{
						if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED || m_points.at(*iterNeighors).clusterID == NOISE)
						{
							if (m_points.at(*iterNeighors).clusterID == UNCLASSIFIED)
							{
								clusterSeeds.push_back(*iterNeighors);
								n = clusterSeeds.size();
							}
							m_points.at(*iterNeighors).clusterID = clusterID;
						}
					}
				}
			}

			return SUCCESS;
		}
	}
	inline double calculateDistance(const PointD& pointCore, const PointD& pointTarget)
	{
		return pow(pointCore.x - pointTarget.x, 2) + pow(10*pointCore.y - 10*pointTarget.y, 2);
	}

	int getTotalPointSize() { return m_pointSize; }
	int getMinimumClusterSize() { return m_minPoints; }
	int getEpsilonSize() { return m_epsilon; }

public:
	std::vector<PointD> m_points;

private:
	unsigned int m_pointSize;
	unsigned int m_minPoints;
	double m_epsilon;
};
