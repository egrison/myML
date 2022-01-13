
#pragma once

#include <list>
#include <unordered_map>

typedef unsigned int uint;

class RouteKeeper {
public:
	template <class func>
	RouteKeeper(std::vector<Point> const& points, func const& func)
	{
		bool isValidId = true;
		isCostCalculated = false;
		for (uint i = 0; i < points.size(); i++)
			vertex_indexer_[points[i].id] = i;
		if (vertex_indexer_.size() != points.size()) {
			// LOG message: Construction error: Nodes with the same ID found. Route network will be cleaned.
			isValidId = false;
		}

		points_.reserve(points.size());
		make_points_(points);
		for (Point a : points_) {
			for (uint i = 0; i < a.connections.size(); i++) {
				if (vertex_indexer_.count(a.connections[i]) == 0) {
					// LOG message: Warning: Edges associated with unknown nodes were found. Founded edges will be removed.
					a.connections.erase(a.connections.begin() + i);
					i--;
				}
			}
		}
		if (isValidId) {
			d_points_.reserve(points.size());
			d_points_.resize(points.size());
			edges_.reserve(points.size());
			edges_.resize(points.size());
			add_edges_(points_, func);
		}
		else {
			points_.clear();
			vertex_indexer_.clear();
		}
	}

	std::vector<Point> get_path(uint from, uint to)
	{
		std::vector<Point> res_path;
		auto to_it = vertex_indexer_.find(to);
		auto from_it = vertex_indexer_.find(from);

		if (to_it == vertex_indexer_.end() || from_it == vertex_indexer_.end()) {
			// LOG message: Message: An attempt to find a path to nodes not included in the route network.
			return res_path;
		}

		bool needReCalcCosts = false;
		if (!isCostCalculated || from != start_vertex)
			needReCalcCosts = true;
		if (d_points_[to_it->second].type != DVType::VISITED)
			needReCalcCosts = true;
		if (needReCalcCosts)
			cost_calculating(from, to);

		if (d_points_[to_it->second].type != DVType::VISITED) {
			// LOG message: Message: The path to the specified point was not found.
			return res_path;
		}

		int current_wp_id = to_it->second;
		while (true) {
			res_path.push_back(points_[current_wp_id]);
			current_wp_id = d_points_[current_wp_id].prev;
			if (current_wp_id == -1)
				break;
		}
		std::reverse(res_path.begin(), res_path.end());
		return res_path;
	}

private:
	struct Edge;
	struct DV;

	uint start_vertex;

	bool isCostCalculated;

	std::vector<DV> d_points_;

	std::vector<Point> points_;

	std::unordered_map<uint, uint> vertex_indexer_;

	std::vector<std::vector<Edge>> edges_;

	struct Edge {
		Edge() {}
		Edge(uint to, double weight)
		{
			this->to = to;
			this->weight = weight;
		}
		uint to = 0;
		double weight = 0;
	};

	enum class DVType { VISITED, TOUCHED, UNTOUCHED };

	struct DV {
		DVType type = DVType::UNTOUCHED;
		int prev = -1;
		double cost = std::numeric_limits<double>::infinity();
	};

	void cost_calculating(uint from, uint to)
	{
		for (uint i = 0; i < d_points_.size(); i++) {
			d_points_[i].type = DVType::UNTOUCHED;
			d_points_[i].cost = std::numeric_limits<double>::infinity();
			d_points_[i].prev = -1;
		}
		if (points_.empty())
			return;
		start_vertex = from;
		uint from_interior = vertex_indexer_[from];
		d_points_[from_interior].type = DVType::TOUCHED;
		d_points_[from_interior].cost = 0;

		std::list<uint> touched;
		touched.push_back(from_interior);

		while (!touched.empty()) {
			uint cur_v = touched.front();
			touched.pop_front();
			d_points_[cur_v].type = DVType::VISITED;
			if (cur_v == vertex_indexer_[to]) {
				break;
			}
			for (uint i = 0; i < edges_[cur_v].size(); i++) {
				uint target_id = edges_[cur_v][i].to;
				switch (d_points_[target_id].type) {
				case DVType::UNTOUCHED: {
					d_points_[target_id].prev = cur_v;
					d_points_[target_id].cost = d_points_[cur_v].cost + edges_[cur_v][i].weight;
					d_points_[target_id].type = DVType::TOUCHED;
					insert_vertex_(touched, d_points_, target_id);
					break;
				}
				case DVType::TOUCHED: {
					if (d_points_[target_id].cost > d_points_[cur_v].cost + edges_[cur_v][i].weight) {
						d_points_[target_id].prev = cur_v;
						d_points_[target_id].cost = d_points_[cur_v].cost + edges_[cur_v][i].weight;
						update_vertex_(touched, d_points_, target_id);
					}
					break;
				}
				default:
					break;
				}
			}
		}
		isCostCalculated = true;
	}

	template <class func>

	void add_edges_(std::vector<Point> const& points, func const& func)
	{
		for (uint i = 0; i < points.size(); i++) {
			edges_[i].reserve(points[i].connections.size());
			edges_[i].resize(points[i].connections.size());
			for (uint j = 0; j < edges_[i].size(); j++) {
				uint aaa = edges_[i].size();
				uint interior_id = vertex_indexer_[points[i].connections[j]];
				double weight = func(points[i], points[interior_id]);
				Edge e;
				e.to = interior_id;
				e.weight = weight;
				edges_[i][j] = e;
			}
		}
	}

	void make_points_(std::vector<Point> const& points)
	{
		for (uint i = 0; i < points.size(); i++) {
			points_.push_back(points[i]);
		}
	}

	void insert_vertex_(std::list<uint>& list, std::vector<DV> const& dv, uint id)
	{
		if (list.empty()) {
			list.push_back(id);
			return;
		}
		for (std::list<uint>::iterator it = list.begin(); it != list.end(); ++it) {
			if (dv[id].cost < dv[*it].cost) {
				list.insert(it, id);
				return;
			}
		}
		list.push_back(id);
	}

	void update_vertex_(std::list<uint>& list, std::vector<DV> const& dv, uint id)
	{
		for (std::list<uint>::iterator it = list.begin(); it != list.end(); ++it) {
			if (*it == id) {
				list.erase(it);
				break;
			}
		}
		insert_vertex_(list, dv, id);
	}
};

