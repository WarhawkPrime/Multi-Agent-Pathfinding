#pragma once

#include "Adapted_DStar_Light_Graphstructures.h"

#include <queue>




// struct for agent
struct Agent
{
	Agent() {};
	Agent(int id, Point coords) : id{ id }, current_coords{ coords } {};
	Agent(int id, int pos) : id{ id }, node_position{ pos } {};
	int id;
	int node_position;
	Point current_coords;
};


// simple helper function to get the minimun of 2 values
//template <typename U>
double min(const double& v1, const double& v2)
{
	if (v1 <= v2)
		return v1;
	else
		return v2;
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

// calc key
// std::shared_ptr<Vertex>
template <typename T_graph>
void calc_key(T_graph& graph, const double& k_m, const int& node, const int& start)
{
	double h;
	calc_heuristic(graph, node, start, 1, h);
	graph.vertex_map.at(node).get()->cost_h = h;
	// min(g(node), rhs(node)) + h(s_start, node) + km
	graph.vertex_map.at(node).get()->key.first = graph.vertex_map.at(node).get()->get_min_g_rhs() + graph.vertex_map.at(node).get()->cost_h + k_m;

	// min(g(node), rhs(node))
	graph.vertex_map.at(node).get()->key.second = graph.vertex_map.at(node).get()->get_min_g_rhs();

	//std::cout << "calc key first : " << key.first << " calc key second : " << key.second << std::endl;
}

template <typename T_key>
bool compare_key(T_key& k0, T_key& k1)
{
	return ((k0.first < k1.first) || (k0.first == k1.first && k0.second > k1.second));
}



bool queue_comp_key(std::shared_ptr<Vertex>& n0, std::shared_ptr<Vertex>& n1)
{
	return ((n0.get()->key.first > n1.get()->key.first) || (n0.get()->key.first == n1.get()->key.first && n0.get()->key.second > n1.get()->key.second));
}



struct Compare_Vertices
{
	bool operator()(std::shared_ptr<Vertex>& n0, std::shared_ptr<Vertex>& n1)
	{
		return (queue_comp_key(n0, n1));
	}
};


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

//find 
template <typename T_graph>
void find_arg_min(T_graph& graph, const int& node, int& arg_min)
{
	double min_val = INT_MAX;
	int min_val_id;

	//for all successor. g should be calculated
	for (const auto& edg_ver : graph.adjacency_map.at(node))
	{
		double val = graph.edge_map.at(edg_ver.first).get()->edge_weight + graph.vertex_map.at(edg_ver.second).get()->cost_g;

		if (val < min_val)
		{
			min_val = val;
			min_val_id = edg_ver.second;
		}
	}

	arg_min = min_val_id;
}


template <typename T_agent>
void move_agent(T_agent& agent, int move_to)
{
	agent.node_position = move_to;
}


//==========	==========	==========	==========

// initialize
void initialize_dstarlite()
{
	//init OPEN as openqueue
	//init km = 0;
	//for all nodes s : rhs(s) = g(s) = Infinity
	//rhs(s_goal) = 0
	//OPEN.Insert(s_goal, [h(s_start, s_goal);0])
}


template <typename T_graph>
void calc_rhs_dlite(T_graph& graph, const int& node_id, const int& dst_id, double& rhs)
{
	double min_val = INT_MAX;

	std::vector<std::pair<int, int>> ajd;
	graph.get_adjacencies(node_id, ajd);

	for (const auto& edg_ver : ajd)
	{
		//only allowed g values are non zero (already calculated) or 0 from scr
		//if (graph.vertex_map.at(edg_ver.second).get()->cost_g != 0 || edg_ver.second == dst_id)
		//{
		double val = graph.vertex_map.at(edg_ver.second).get()->cost_g + graph.edge_map.at(edg_ver.first).get()->edge_weight;

		//update min_val if val is smaller.
		if (val < min_val)
		{
			min_val = val;
		}
		//}
	}
	//set calculated rhs. 
	rhs = min_val;
}


// update vertex
template <typename T_graph, typename U_queue>
void update_dstar_vertex(T_graph& graph, U_queue& open_queue, std::set<int>& open_tracker, const int& node_id, const int& src_id, const int& dst_id, const double& k_m)
{
	//if (u != goal)
	if (node_id != dst_id)
	{
		//rhs (u) = min s'element succ(u) (c(u,s') + g(s')
		double rhs;
		calc_rhs_dlite(graph, node_id, dst_id, rhs);
		graph.vertex_map.at(node_id).get()->cost_rhs = rhs;
	}

	// if (u element of OPEN)
	if (open_tracker.contains(node_id))
	{
		//OPEN.Remove(u)
		delete_from_queue(open_queue, node_id);
		open_tracker.erase(node_id);
	}

	//if g(u) != rhs(u)
	if (graph.vertex_map.at(node_id).get()->cost_g != graph.vertex_map.at(node_id).get()->cost_rhs)
	{
		//OPEN Insert(u, CalcKey)
		calc_key(graph, k_m, node_id, src_id);
		open_queue.push(graph.vertex_map.at(node_id));
		open_tracker.insert(node_id);
	}
}

// computing shortest phat
template <typename T_graph, typename U_open, typename V_opentracker >
bool compute_dstarlite_shortest_path(T_graph& graph, U_open& open_queue, V_opentracker& open_tracker, const auto& src_id, const auto& dst_id, double& k_m)
{
	// while (OPEN.TopKey < CalculateKey(s_start) OR rhs(s_tart) > g(s_start))
	// we can check for the goal id
	while (!open_queue.empty())
	{
		if (graph.vertex_map.at(src_id).get()->cost_rhs == graph.vertex_map.at(src_id).get()->cost_g)
		{
			//std::cout << " rhs == g " << std::endl;
			//return true;
		}

		std::pair<double, double> k_old;
		std::pair<double, double> k_new;

		//current = OPEN.Top()
		std::shared_ptr<Vertex> current = open_queue.top();

		//u = OPEN.Pop()
		open_queue.pop();
		open_tracker.erase(current.get()->id);	//dont forget the tracker!


		//check if OPEN.TopKey < Calckey(s_start)
		if (current.get()->id == src_id)
		{
			std::cout << "start found " << std::endl;
			current.get()->cost_g = current.get()->cost_rhs;
			std::cout << "id " << current.get()->id << std::endl;
			std::cout << "g " << current.get()->cost_g << std::endl;
			return true;
		}


		calc_key(graph, k_m, src_id, src_id);
		if (compare_key(graph.vertex_map.at(src_id).get()->key, current.get()->key))
		{
			//std::cout << "src_id key > than current key" << std::endl;
			graph.vertex_map.at(src_id).get()->cost_g = graph.vertex_map.at(src_id).get()->cost_rhs;
			return true;
		}

		//k_old = OPEN.TopKey()
		k_old = current.get()->key;

		//k_Top = CalculateKey(current)
		calc_key(graph, k_m, current.get()->id, src_id);
		k_new = current.get()->key;


		//if(k_old < k_new)
		if (compare_key(k_old, k_new))
		{
			//	OPEN.Insert(u, CalculateKey(u))
			current.get()->key = k_new;
			open_queue.push(current);
			open_tracker.insert(current.get()->id);
		}
		//else if (g(current) > rhs(current))
		else if (current.get()->cost_g > current.get()->cost_rhs)
		{
			// g(current) = rhs(current)
			current.get()->cost_g = current.get()->cost_rhs;

			// for all nodes Element of Predecessors (current)
			for (auto& edg_ver : graph.adjacency_map.at(current.get()->id))
			{
				update_dstar_vertex(graph, open_queue, open_tracker, edg_ver.second, src_id, dst_id, k_m);
			}
		}
		else
		{
			//g(current) = infinity
			current.get()->cost_g = INT_MAX;

			//UpdateVertex(current)
			update_dstar_vertex(graph, open_queue, open_tracker, current.get()->id, src_id, dst_id, k_m);

			//all neighbours/predecessors
			for (auto& edg_ver : graph.adjacency_map.at(current.get()->id))
			{
				//UpdateVertex(s)
				update_dstar_vertex(graph, open_queue, open_tracker, edg_ver.second, src_id, dst_id, k_m);
			}
		}
	}
	std::cout << " returned false, start could not been found " << std::endl;
	return false;
}


//dstar lite main function
template <typename T_graph>
bool dstarlite_main(T_graph& graph, int s, int d)
{
	int src_id = s;
	int dst_id = d;

	int src_start = src_id;
	int src_last;

	Agent agent(0, src_start);

	//s_last = s_start
	src_last = src_start;

	//initialize changing iterations
	std::vector< std::vector<std::pair<int, double>> > global_changes;
	change_environment(global_changes);

	//========== initialize ==========
	// OPEN = EMPTY
	// k_m = 0;
	// all nodes rhs = g = INFINITY. Before a node is in any way checked, set rhs and g to Infinity
	//rhs s_goal = 0
	// OPEN.Insert s_goal, Calckey)

	std::priority_queue<std::shared_ptr<Vertex>, std::vector<std::shared_ptr<Vertex>>, Compare_Vertices> open_queue;
	std::set<int> open_tracker;

	//est distance. heuristical dist. mean/mittelwert
	double k_m = 0;

	//rhs(s_goal) = 0
	graph.vertex_map.at(dst_id).get()->cost_rhs = 0;

	//calc key for src_start
	graph.vertex_map.at(dst_id).get()->cost_h = 0;
	calc_key(graph, k_m, dst_id, dst_id);

	//OPEN.insert(s_goal, key(goal))
	open_queue.push(graph.vertex_map.at(dst_id));
	open_tracker.insert(dst_id);

	//ComputeShortestPath
	compute_dstarlite_shortest_path(graph, open_queue, open_tracker, src_start, dst_id, k_m);

	std::vector< std::vector<std::pair<int, double>> > changes;
	change_environment(changes);

	//while s_start != s_goal
	while (src_start != dst_id)
	{
		// if rhs(s_start = INFINITY), then there is no known path
		if (graph.vertex_map.at(src_start).get()->cost_rhs == INT_MAX)
		{
			std::cout << " you shall not path! " << std::endl;
			return false;
		}

		//s_start = arg min s'element of successor (s_start) ( c(s_start, s') + g(s')). arg min is the node for which the minimum is met.
		int arg_min;
		find_arg_min(graph, src_start, arg_min);
		src_start = arg_min;

		//move to start
		move_agent(agent, src_start);
		std::cout << "move agent to : " << src_start << std::endl;

		for (size_t change_timings = 0; change_timings < global_changes.size(); change_timings++)
		{
			std::cout << " changes " << std::endl;
			//k_m = k_m + h(s_last, s_start)
			double h;
			calc_heuristic(graph, src_last, src_start, 1, h);
			k_m = k_m + h;

			//s_last = s_start
			src_last = src_start;

			//for all edges(u,v) with changed edge costs
			for (auto& node_weight : changes.at(change_timings))
			{
				for (auto& edg_ver : graph.adjacency_map.at(node_weight.first))
				{
					// Update the edge cost c(u,v)
					graph.edge_map.at(edg_ver.first).get()->edge_weight = node_weight.second;

					//update_dstar_vertex(graph, open_queue, open_tracker, edg_ver.second, src_id, dst_id, k_m);
				}
				//update vertex (u)
				update_dstar_vertex(graph, open_queue, open_tracker, node_weight.first, src_id, dst_id, k_m);
			}
			compute_dstarlite_shortest_path(graph, open_queue, open_tracker, src_start, dst_id, k_m);
		}
		//scan graph for changed edge costs
		//if any edge costs changed


		/*
		It
		does not make any assumptions about how the edge costs
		change, whether they go up or down, whether they change
		close to the current vertex of the robot or far away from it, or
		whether they change in the world or only because the robot
		revised its initial estimates
		*/

		/*
		if(t)
		{
			std::cout << "changes " << std::endl;
			//k_m = k_m + h(s_last, s_start)
			double h;
			calc_lpa_heuristic(graph, src_last, src_start, 1, h);
			k_m = k_m + h;

			//s_last = s_start
			src_last = src_start;

			//for all edges(u,v) with changed edge costs
				//std::vector<std::pair<int, int>> node_weights;


			for (auto& node_weight : changes.at(change_timing))
			{
				for (auto& edg_ver : graph.adjacency_map.at(node_weight.first))
				{
					// Update the edge cost c(u,v)
					graph.edge_map.at(edg_ver.first).get()->edge_weight = node_weight.second;

					//update_dstar_vertex(graph, open_queue, open_tracker, edg_ver.second, src_id, dst_id, k_m);
				}
				//update vertex (u)
				update_dstar_vertex(graph, open_queue, open_tracker, node_weight.first, src_id, dst_id, k_m);

			}

			//change_timing++;
			compute_dstarlite_shortest_path(graph, open_queue, open_tracker, src_start, dst_id, k_m);
		}

		//if (change_timing == changes.size() -1)
		if (change_timing == changes.size()-1)
			t = false;

		change_timing++;
		*/
	}

	std::cout << " end " << std::endl;
	return true;
}