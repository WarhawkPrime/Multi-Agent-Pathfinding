#pragma once

#include <queue>
#include <list>
#include <algorithm>
#include <vector>
#include <iostream>

#include <utility>
#include <functional>

#include <unordered_map>
#include <map>

#include "Adapted_LPAStar_Graphstructures.h"

//========== Algorithm functions

template <typename T_graph>
void calc_key(T_graph& graph, const int& node_id)
{
	graph.vertex_map.at(node_id).get()->key.first = graph.vertex_map.at(node_id).get()->get_min_g_rhs() + graph.vertex_map.at(node_id).get()->cost_h;
	graph.vertex_map.at(node_id).get()->key.second = graph.vertex_map.at(node_id).get()->get_min_g_rhs();
};

/*
void calc_key(std::shared_ptr<Vertex> node, std::pair<double, double>& key)
{
	key.first = node.get()->get_min_g_rhs() + node.get()->cost_h;
	key.second = node.get()->get_min_g_rhs();
	//std::cout << "calc key first : " << key.first << " calc key second : " << key.second << std::endl;
}
*/

bool comp_keys(std::shared_ptr<Vertex>& n0, std::shared_ptr<Vertex>& n1)
{
	return ((n0.get()->key.first < n1.get()->key.first) || (n0.get()->key.first == n1.get()->key.first && n0.get()->key.second <= n1.get()->key.second));
}



bool queue_comp_key(std::shared_ptr<Vertex>& n0, std::shared_ptr<Vertex>& n1)
{
	return ((n0.get()->key.first > n1.get()->key.first) || (n0.get()->key.first == n1.get()->key.first && n0.get()->key.second > n1.get()->key.second));
}



struct Compare_Vertexs
{
	bool operator()(std::shared_ptr<Vertex>& n0, std::shared_ptr<Vertex>& n1)
	{
		return (queue_comp_key(n0, n1));
	}
};



//function to calc rhs.
template <typename T_graph>
void calc_rhs(T_graph& graph, const int& node_id, const int& src_id, double& rhs)
{
	//rhs =min value of (for all neighs g + edge weight.)
	double min_val = INT_MAX;

	std::vector<std::pair<int, int>> ajd;
	graph.get_adjacencies(node_id, ajd);

	for (const auto& edg_ver : ajd)
	{
		//only allowed g values are non zero (already calculated) or 0 from scr
		if (graph.vertex_map.at(edg_ver.second).get()->cost_g != 0 || edg_ver.second == src_id)
		{

			double val = graph.vertex_map.at(edg_ver.second).get()->cost_g + graph.edge_map.at(edg_ver.first).get()->edge_weight;

			//update min_val if val is smaller.
			if (val < min_val)
			{
				min_val = val;
			}
		}
	}

	//set calculated rhs. 
	rhs = min_val;
}



// adapt the heuristic later. maybe change h with alpha. later with compressed DH?
template <typename T_graph>
void calc_lpa_heuristic(const T_graph& env, int node_id_0, int node_id_1, double alpha, double& heuristic)
{
	double dx = abs(env.get_x_coord(node_id_0) - env.get_x_coord(node_id_1));
	double dy = abs(env.get_y_coord(node_id_0) - env.get_y_coord(node_id_1));
	double dz = abs(env.get_z_coord(node_id_0) - env.get_z_coord(node_id_1));
	heuristic = { sqrt(dx * dx + dy * dy + dz * dz) };
}



template <typename T_queue>
void delete_from_queue(T_queue& queue, const int& id_to_delete)
{
	T_queue temp_queue;

	while (!queue.empty())
	{
		auto current = queue.top();
		queue.pop();
		if (current.get()->id != id_to_delete)
			temp_queue.push(current);
	}

	queue.swap(temp_queue);
}



template <typename T_graph, typename U_queue>
void update_vertex(T_graph& graph, U_queue& queue, std::set<int>& open_tracker, const int& neighbour_node_id, const int& src_id, const int& dst_id)
{
	//if node is not start
	//if ( node_id != start_id )
	if (neighbour_node_id != src_id)
	{
		double rhs = 0;
		calc_rhs(graph, neighbour_node_id, src_id, rhs);
		graph.vertex_map.at(neighbour_node_id).get()->cost_rhs = rhs;
	}


	//remove from open_queue
	if (open_tracker.contains(neighbour_node_id))
	{
		delete_from_queue(queue, neighbour_node_id);
		open_tracker.erase(neighbour_node_id);
	}


	//insert to queue if locally inconsistent, key gets calculated within the queue
	//only here calc g and h !!!
	if (graph.vertex_map.at(neighbour_node_id).get()->cost_g != graph.vertex_map.at(neighbour_node_id).get()->cost_rhs)
	{
		double g = INT_MAX;
		double h = 0;

		calc_lpa_heuristic(graph, neighbour_node_id, dst_id, 1, h);

		graph.vertex_map.at(neighbour_node_id).get()->cost_g = g;
		graph.vertex_map.at(neighbour_node_id).get()->cost_h = h;

		//calc key
		calc_key(graph, neighbour_node_id);

		queue.push(graph.vertex_map.at(neighbour_node_id));
		open_tracker.insert(neighbour_node_id);
	}
}

/*


template <typename T_graph >
bool LPA_shortest_path(T_graph& graph)
{
	//queue holds all vertices locally inconsistent ( cost_g != cost_rhs )
	std::priority_queue<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Vertex>>, Compare_Vertexs> open_queue;

	//set to be identical with open, only use it to check for "contain" in open_queue
	std::set<int> open_tracker;

	//initialize src and dst
	int src_id = 1;
	int dst_id = 26;

	//initialize source_node
	graph.vertex_map.at(src_id).get()->cost_g = INT_MAX;
	graph.vertex_map.at(src_id).get()->cost_rhs = 0;
	double start_h;
	calc_lpa_heuristic(graph, src_id, dst_id, 1, start_h);

	graph.vertex_map.at(src_id).get()->cost_h = start_h;

	//push start in OPEN
	open_queue.push(graph.vertex_map.at(src_id));
	open_tracker.insert(src_id);

	//if all vertices are local consistent
	while (!open_queue.empty())
	{
		//current = OPEN.extract_min()
		std::shared_ptr<Vertex> current = open_queue.top();

		//directly pop it from OPEN
		open_queue.pop();
		open_tracker.erase(current.get()->id);

		// if key(goal) is top of OPEN, the same if rhs and g of goal are inconsistent
		if (current.get()->id == dst_id)
		{
			std::cout << "goal found " << std::endl;

			current.get()->cost_g = current.get()->cost_rhs;

			std::cout << "id " << current.get()->id << std::endl;
			std::cout << "g " << current.get()->cost_g << std::endl;
			return true;
		}

		//differs from astar
		if (current.get()->cost_g >= current.get()->cost_rhs)
		{
			current.get()->cost_g = current.get()->cost_rhs;

			//foreach vertex v element of Adj(u) do:
			std::vector<std::pair<int, int>> ajd;
			graph.get_adjacencies(current.get()->id, ajd);

			//update every neighbour
			for (const auto& edg_ver : ajd)
			{
				update_vertex(graph, open_queue, open_tracker, edg_ver.second, src_id, dst_id);
			}

		}
		//vertex inconsistent, neighbour miminum is smaller than own min(g,rhs)
		else
		{
			std::cout << "inc" << std::endl;
			current.get()->cost_g = INT_MAX; //infinity

			//update current
			update_vertex(graph, open_queue, open_tracker, current.get()->id, src_id, dst_id);

			//update every neighbour
			std::vector<std::pair<int, int>> ajd;
			graph.get_adjacencies(current.get()->id, ajd);
			for (const auto& edg_ver : ajd)
			{
				update_vertex(graph, open_queue, open_tracker, edg_ver.second, src_id, dst_id);
			}
		}
	}
	return false;
}


*/


template <typename T_graph, typename U_open, typename V_opentracker >
bool LPA_shortest_path_pro(T_graph& graph, U_open& open_queue, V_opentracker& open_tracker, const auto& src_id, const auto& dst_id)
{
	//if all vertices are local consistent
	while (!open_queue.empty()) //|| ( graph.vertex_map.at(dst).get()->cost_g == 0 ||  goal rhs != goal g)muss noch angepasst werden
	{
		//current = OPEN.extract_min()
		std::shared_ptr<Vertex> current = open_queue.top();

		//directly pop it from OPEN
		open_queue.pop();
		open_tracker.erase(current.get()->id);

		// if key(goal) is top of OPEN, the same if rhs and g of goal are inconsistent
		if (current.get()->id == dst_id)
		{
			std::cout << "goal found " << std::endl;

			current.get()->cost_g = current.get()->cost_rhs;

			std::cout << "id " << current.get()->id << std::endl;
			std::cout << "g " << current.get()->cost_g << std::endl;
			return true;
		}



		//differs from astar
		if (current.get()->cost_g >= current.get()->cost_rhs)
		{
			current.get()->cost_g = current.get()->cost_rhs;

			//foreach vertex v element of Adj(u) do:
			std::vector<std::pair<int, int>> ajd;
			graph.get_adjacencies(current.get()->id, ajd);

			//update every neighbour
			for (const auto& edg_ver : ajd)
			{
				update_vertex(graph, open_queue, open_tracker, edg_ver.second, src_id, dst_id);
			}

		}
		//vertex inconsistent, neighbour miminum is smaller than own min(g,rhs)
		else
		{
			std::cout << "inc" << std::endl;
			current.get()->cost_g = INT_MAX; //infinity

			//update current
			update_vertex(graph, open_queue, open_tracker, current.get()->id, src_id, dst_id);

			//update every neighbour
			std::vector<std::pair<int, int>> ajd;
			graph.get_adjacencies(current.get()->id, ajd);
			for (const auto& edg_ver : ajd)
			{
				update_vertex(graph, open_queue, open_tracker, edg_ver.second, src_id, dst_id);
			}
		}
	}
	return false;
};


template <typename T_graph>
void change_vertex(T_graph& graph, int& node_id, double& edge_weights)
{
	for (auto& edg_ver : graph.adjacency_map.at(node_id))
	{
		graph.edge_map.at(edg_ver.first).get()->edge_weight = edge_weights;
	}
}

//Main Funtion of Lifelong planning astar
template <typename T_graph>
void LPA_Mainfunction(T_graph& graph)
{
	//queue holds all vertices locally inconsistent ( cost_g != cost_rhs )
	std::priority_queue<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Vertex>>, Compare_Vertexs> open_queue;

	//set to be identical with open, only use it to check for "contain" in open_queue
	std::set<int> open_tracker;

	int src_id = 1;
	int dst_id = 8;

	//initialize source_node
	graph.vertex_map.at(src_id).get()->cost_g = INT_MAX;
	graph.vertex_map.at(src_id).get()->cost_rhs = 0;
	double start_h;
	calc_lpa_heuristic(graph, src_id, dst_id, 1, start_h);

	graph.vertex_map.at(src_id).get()->cost_h = start_h;

	//initialize dest_node
	graph.vertex_map.at(dst_id).get()->cost_g = INT_MAX;

	//initialize changing iterations
	std::vector< std::vector<std::pair<int, double>> > global_changes;
	change_environment(global_changes);

	//push start in OPEN
	open_queue.push(graph.vertex_map.at(src_id));
	open_tracker.insert(src_id);

	int c = 0;
	

	for (auto& vec : global_changes)
	{
		LPA_shortest_path_pro(graph, open_queue, open_tracker, src_id, dst_id);
		//wait for changes in edge costs

		std::cout << " before update global changes " << std::endl;
		std::cout << " goal g : " << graph.vertex_map.at(dst_id).get()->cost_g << std::endl;

		for (auto& node_weight : vec)
		{
			change_vertex(graph, node_weight.first, node_weight.second);
		}
	}

	std::cout << " end " << std::endl;
	std::cout << std::endl;
	std::cout << " goal g : " << graph.vertex_map.at(dst_id).get()->cost_g << std::endl;

}