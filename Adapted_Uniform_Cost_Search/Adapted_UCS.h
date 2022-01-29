#pragma once

#include "Adapted_UCS_Graphstructures.h"

#include <unordered_map>
#include <unordered_set>
#include <iterator>
#include <vector>
#include <set>
#include <tuple>
#include <iostream>
#include <utility>
#include <functional>
#include <list>
#include <algorithm>
#include <queue>


bool ucs(Graph env)
{
	//initialitaion of src and dst. must be changed for vector? for multiple src/dst.
	int src_id = 0;
	int dst_id = 8;

	//initialize openlist as a heap (queue) should hold the nodes or ids ?
	std::priority_queue<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Vertex>>, Compare_Nodes> open_queue;

	//set to be identical with open
	std::set<int> open_tracker;

	//closed list to check if already visited
	std::set<int> closed_set;

	//OPEN.insert(s)
	open_queue.push(env.vertex_map.at(src_id));
	open_tracker.insert(src_id);

	//while OPEN =/= 0 do:
	while (!open_queue.empty())
	{

		if (open_queue.top().get()->id != dst_id)
			break;

		//current = OPEN.extract_min()
		std::shared_ptr<Vertex> current = open_queue.top();

		//foreach vertex v element of Adj(u) do:
		for (auto edg_ver : env.get_adjacencies(current.get()->id))
		{
			//calculate the g costs
			double new_costs = current.get()->cost_g + env.get_edge(edg_ver.first).get()->edge_weight;

			//check if neighbour is in closed. if true, ignore it
			if (!closed_set.count(env.get_node(edg_ver.second).get()->id))
			{
				//check if neighbour is in open
				if (open_tracker.count(env.get_node(edg_ver.second).get()->id)) {

					//if true, check if the new costs are smaller than the already calculated costs. if true, replace
					if (env.get_node(edg_ver.second).get()->cost_g > new_costs)
						env.get_node(edg_ver.second).get()->cost_g = new_costs;
				}
				else
				{
					//if false, set the new costs and put in the open_queue
					env.get_node(edg_ver.second).get()->cost_g = new_costs;

					open_queue.push(env.get_node(edg_ver.second));
					open_tracker.insert(env.get_node(edg_ver.second).get()->id);
				}
			}
		}
		//pop current from open_queue and put it in the closed list
		open_queue.pop();
		open_tracker.erase(current.get()->id);
		closed_set.insert(current.get()->id);
	}

	std::cout << "ucs goal found " << std::endl;
	std::cout << "costs to goal: " << env.get_g_costs(dst_id) << std::endl;

	return true;
}