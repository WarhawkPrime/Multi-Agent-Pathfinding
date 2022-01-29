#pragma once

#include "Adapted_AStar_Graphstructures.h"

#include <queue>
#include <list>
#include <algorithm>
#include <vector>
#include <iostream>

#include <utility>
#include <functional>

#include <unordered_map>
#include <map>

void calc_g(const Graph& env, double g_cost, int edge_id, double& new_g_cost)
{
	new_g_cost = { g_cost + env.edge_map.at(edge_id).get()->edge_weight };
}

// adapt the heuristic later. maybe change h with alpha. later with compressed DH?
template <typename T_graph>
void calc_heuristic(const T_graph& env, int node_id_0, int node_id_1, double alpha, double& heuristic)
{
	double dx = abs(env.get_x_coord(node_id_0) - env.get_x_coord(node_id_1));
	double dy = abs(env.get_y_coord(node_id_0) - env.get_y_coord(node_id_1));
	double dz = abs(env.get_z_coord(node_id_0) - env.get_z_coord(node_id_1));
	heuristic = { sqrt(dx * dx + dy * dy + dz * dz) };
}


template <typename T, typename U, typename V, typename W>
bool tilt_out_bucket(T& env, U& open_queue, V& open_tracker, W& open_bucket, double& bucket_cap)
{
	//fill openqueue if its empty and open_bucket has still members

	//fill openqueue with bucket-items. Then the circle begins again.
	double min_f_cost = 0;
	for (auto it = open_bucket.begin(); it != open_bucket.end(); ++it)
	{
		if (min_f_cost == 0 || it->first < min_f_cost) {
			min_f_cost = it->first;
		}
	}
	//get range of f_costs.
	std::vector<std::pair<double, int>> keys;
	for (auto it = open_bucket.begin(); it != open_bucket.end(); ++it)
	{
		if (it->first <= (min_f_cost * bucket_cap))
		{
			keys.push_back(std::make_pair(it->first, it->second));
		}
	}
	//fill openqueue
	for (const auto& elem : keys)
	{
		open_queue.push(env.get_node(elem.second));
		open_tracker.insert(env.get_node(elem.second).get()->id);
		open_bucket.erase(elem.first);
	}
	return true;
}

bool astar_hotqueue(Graph env)
{
	//initialitaion of src and dst. must be changed for vector? for multiple src/dst.
	int src_id = 1;
	int dst_id = 26;

	//configure_3D_Graph(env, src_id, dst_id);

	//initialize openlist as a heap (queue) should hold the nodes or ids ?
	std::priority_queue<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Vertex>>, Compare_Nodes> open_queue;

	//set to be identical with open
	std::set<int> open_tracker;

	//bucket to hold "bad" open nodes which we probably dont need
	std::unordered_multimap<double, int> open_bucket;
	const double bucket_cap_mod = 1.25;
	double bucket_cap = 0;	//value to limit which nodes are good or bad

	//closed list to check if already visited
	std::set<int> closed_set;

	//OPEN.insert(s)
	open_queue.push(env.vertex_map.at(src_id));
	open_tracker.insert(src_id);

	//while OPEN =/= 0 do:
	while (!open_queue.empty())
	{
		//current = OPEN.extract_min()
		std::shared_ptr<Vertex> current = open_queue.top();

		//directly pop current from queue
		open_queue.pop();
		open_tracker.erase(current.get()->id);

		std::cout << " open queue size : " << open_queue.size() << std::endl;
		//std::cout << " bucket size " << open_bucket.size() << std::endl;

		if (current.get()->id == dst_id)
		{
			std::cout << "goal found " << std::endl;

			std::cout << "id " << current.get()->id << std::endl;
			std::cout << "g " << current.get()->cost_g << std::endl;
			return true;
		}

		//foreach vertex v element of Adj(u) do:
		std::vector<std::pair<int, int>> ajd;
		env.get_adjacencies(current.get()->id, ajd);

		for (auto& edg_ver : ajd)
		{
			//calculate the 3 costs
			double g_cost = 0;
			double h_cost = 0;
			calc_g(env, current.get()->cost_g, edg_ver.first, g_cost);
			calc_heuristic(env, edg_ver.second, dst_id, 1, h_cost);
			double f_cost = g_cost + h_cost;

			//adapt bucket_cap
			if (bucket_cap == 0 || f_cost < (bucket_cap / bucket_cap_mod))
			{
				//std::cout << "f_costs: " << f_cost << " * " << bucket_cap_mod << std::endl;
				bucket_cap = f_cost * bucket_cap_mod;
				//std::cout << " bucket_cap " << bucket_cap << std::endl;
			}

			//check if neighbour is in closed. if true, ignore it
			if (!closed_set.count(env.get_node(edg_ver.second).get()->id))
			{
				//check if neighbour is in open
				if (open_tracker.count(env.get_node(edg_ver.second).get()->id)) {

					//if in open, check if the new costs are smaller than the already calculated costs. if true, replace
					if (env.get_f_costs(edg_ver.second) > f_cost)
					{
						env.set_g_cost(edg_ver.second, g_cost);
						env.set_h_cost(edg_ver.second, h_cost);
						env.set_f_cost(edg_ver.second, f_cost);
					}
				}
				else
				{
					//if not in open and not in closed, set the costs and put neighbour in the open_queue
					env.set_g_cost(edg_ver.second, g_cost);
					env.set_h_cost(edg_ver.second, h_cost);
					env.set_f_cost(edg_ver.second, f_cost);


					//decide if pushed in open or open_bucket
					if (f_cost <= bucket_cap)
					{
						//std::cout << " push to open " << std::endl;
						open_queue.push(env.get_node(edg_ver.second));
						open_tracker.insert(env.get_node(edg_ver.second).get()->id);
					}
					{
						open_bucket.insert(std::make_pair(f_cost, edg_ver.second));
					}
				}
			}
		}
		//put current in openlist
		closed_set.insert(current.get()->id);

		//fill openqueue if its empty and open_bucket has still members
		if (open_queue.empty() && !open_bucket.empty())
		{
			tilt_out_bucket(env, open_queue, open_tracker, open_bucket, bucket_cap);
		}
	}
	std::cout << " no goal found " << std::endl;
	return false;
}