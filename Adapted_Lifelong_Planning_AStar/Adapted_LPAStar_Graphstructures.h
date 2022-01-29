#pragma once

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

namespace constants
{
	constexpr int adjacent_number = 26;
}

typedef typename std::tuple<double, double, double> Point;

std::set<Point> directions = {
Point(0, 1, 0),		//North
Point(1, 1, 0),		//NorthEast
Point(1, 0, 0),		//East
Point(1, -1, 0),	//SouthEast
Point(0, -1, 0),	//South
Point(-1, -1, 0),	//SouthWest
Point(-1, 0, 0),	//West
Point(-1, 1, 0),	//NorthWest
Point(0, 0, 1),		//UpMiddle
Point(0, 1, 1),		//UpNorth
Point(1, 1, 1),		//UpNorthEast
Point(1, 0, 1),		//UpEast
Point(1, -1, 1),	//UpSouthEast
Point(0, -1, 1),	//UpSouth
Point(-1, -1, 1),	//UpSouthWest
Point(-1, 0, 1),	//UpWest
Point(-1, 1, 1),	//UpNorthWest
Point(0, 0, -1),	//DownMiddle
Point(0, 1, -1),	//DownNorth
Point(1, 1, -1),	//DownNorthEast
Point(1, 0, -1),	//DownEast
Point(1, -1, -1),	//DownSouthEast
Point(0, -1, -1),	//DownSouth
Point(-1, -1, -1),	//DownSouthWest
Point(-1, 0, -1),	//DownWest
Point(-1, 1, -1),	//DownNorthWest
};


struct Vertex
{
	Vertex() {};
	Vertex(int id) : id{ id } {};

	int id;
	double cost_g = INT_MAX;
	double cost_h = 0;
	double cost_rhs = INT_MAX;

	Point coord;
	std::pair<double, double> key;

	//	k1(vertex) = min(g(vertex), rhs(vertex)) + h(vertex)
	double get_min_g_rhs()
	{
		if (this->cost_g <= cost_rhs)
			return this->cost_g;
		else
			return this->cost_rhs;
	}
};



struct Edge
{
	Edge() {};
	Edge(int id) : id{ id } { this->edge_weight = 1; };
	Edge(int id, double edge_weight) : id{ id }, edge_weight{ edge_weight } {};

	int id;
	double edge_weight;
};



class Graph
{
public:
	Graph() {};

	//map with all vertexs, the key is the id.
	//map with all edges, the key is the id
	//set to map two vertexs with an edge. first is id for the vertex, second is vector of (edge_id, vertex_id). vector can be used to get all adj. and iterate through them
	std::unordered_map<int, std::shared_ptr<Vertex>> vertex_map;
	std::unordered_map<int, std::shared_ptr<Edge>> edge_map;
	std::unordered_map<int, std::vector<std::pair<int, int>> > adjacency_map;

	std::vector<int> sources;
	std::vector<int> destinations;

	//getter for vertexs and edges
	std::shared_ptr<Vertex> get_vertex(int vertex_id) { return vertex_map.at(vertex_id); }
	std::shared_ptr<Edge> get_edge(int edge_id) { return edge_map.at(edge_id); }
	void get_adjacencies(int vertex_id, std::vector<std::pair<int, int>>& ajd) { ajd = adjacency_map.at(vertex_id); }

	//getter for coordinates
	//double get_x_coord(int vertex_id) const { return std::get<0>(vertex_map.at(vertex_id).get()  ); }
	double get_x_coord(int vertex_id) const { return std::get<1>(vertex_map.at(vertex_id).get()->coord); }
	double get_y_coord(int vertex_id) const { return std::get<1>(vertex_map.at(vertex_id).get()->coord); }
	double get_z_coord(int vertex_id) const { return std::get<2>(vertex_map.at(vertex_id).get()->coord); }

	//getter setter for values
	double get_g_costs(int vertex_id) const { return vertex_map.at(vertex_id).get()->cost_g; }
	void set_g_cost(int vertex_id, double new_cost_g) { vertex_map.at(vertex_id).get()->cost_g = new_cost_g; }

	double get_h_costs(int vertex_id) const { return vertex_map.at(vertex_id).get()->cost_h; }
	void set_h_cost(int vertex_id, double new_cost_h) { vertex_map.at(vertex_id).get()->cost_h = new_cost_h; }

	double get_rhs_costs(int vertex_id) const { return vertex_map.at(vertex_id).get()->cost_rhs; }
	void set_rhs_cost(int vertex_id, double new_cost_rhs) { vertex_map.at(vertex_id).get()->cost_rhs = new_cost_rhs; }
};


//set to check which directions have been used. fill it with 
void build_directions(std::set<Point>& possible_directions, const Point& own_coords)
{
	//generate all 26  possible neighbour coordinates based on the own coordiantes.
	for (const auto& vect : directions)
	{
		double d_x = std::get<0>(vect) + std::get<0>(own_coords);
		double d_y = std::get<1>(vect) + std::get<1>(own_coords);
		double d_z = std::get<2>(vect) + std::get<2>(own_coords);

		Point d_point(d_x, d_y, d_z);
		possible_directions.insert(d_point);
	}
}

template <typename T_Graph>
void map_vertices(T_Graph& graph, int& vertex_id, int& edge_ids_taken, int& vertex_ids_taken)
{
	//create adjacencie vector holding all adjacencies for the vertex vertex_id
	std::vector<std::pair<int, int>> adjacencies;

	//sets to hold all possible neighbours of vertex_id (26)
	std::set<Point> possible_directions;

	// if id is 0, its the first vertex. all other vertexs are build around it and so already have coordinates
	if (vertex_id == 0)
	{
		graph.vertex_map.at(vertex_id).get()->coord = Point(0, 0, 0);
	}

	//get all possible vertexs current vertex can achieve
	build_directions(possible_directions, graph.vertex_map.at(vertex_id).get()->coord);

	//for every (vertex_id, vector<edge_id, neighbour_id>), search if it lists current as an neighbour
	for (auto& elem : graph.adjacency_map)
	{
		//vector of edge_id, neighbour_id
		bool neigh = false;
		for (auto& ed_ne : elem.second)
		{
			//current vertex is mentioned as neighbour
			if (ed_ne.second == vertex_id)
			{
				//so elem is a neighbour of current with an existing edge
				adjacencies.push_back(std::make_pair(ed_ne.first, elem.first));
				neigh = true;
			}
		}
	}


	//store vertex_ids into a set. Use another set to store id that have to be added after the loop
	std::set<int> adj_vertexs;
	std::set<int> to_add;

	//fill adj set
	for (auto& elem : adjacencies)
	{
		adj_vertexs.insert(elem.second);
	}

	//for all neighbours in adjacencies vector
	for (auto& elem : adjacencies)
	{
		//for all edge_vertexId pairs in the corresponding adjacency list
		for (auto& ed_ne : graph.adjacency_map.at(elem.second))
		{
			//check if direction is possible
			if (possible_directions.contains(graph.vertex_map.at(ed_ne.second).get()->coord))
			{
				//check if id isn't already in the adj list (with the help of the set)
				if (!adj_vertexs.contains(ed_ne.second))
				{
					//insert vertex id to set if no duplicate
					to_add.insert(ed_ne.second);
				}
			}
		}
	}

	//now add the vertexs to the adjacency vector. create edges
	for (auto& id : to_add)
	{
		adjacencies.push_back(std::make_pair(edge_ids_taken, id));
		graph.edge_map.insert(std::make_pair(edge_ids_taken, std::make_shared<Edge>(edge_ids_taken)));
		edge_ids_taken++;
	}

	//now, see if we need to add left vertexs to current vertex.
	//substract adj from possible dirs, the rest are new neighbours. If! they are never before
	//mentioned as a neighbour.
	for (const auto& coord : adjacencies)
	{
		possible_directions.erase(graph.vertex_map.at(coord.second).get()->coord);
	}

	//if vertex doesnt exist, set -1
	if (!graph.vertex_map.contains(vertex_ids_taken))
	{
		vertex_ids_taken = -1;
	}


	//vertex_ids_taken means that no free vertexs exist
	if (vertex_ids_taken != -1)
	{
		//for every possible direction, add free vertex in adj vector, cac its coords
		for (const auto& coord : possible_directions)
		{
			if (vertex_ids_taken != -1)
			{
				//push vertex with new edge, increase counters, set coordinate of vertex
				adjacencies.push_back(std::make_pair(edge_ids_taken, vertex_ids_taken));
				graph.edge_map.insert(std::make_pair(edge_ids_taken, std::make_shared<Edge>(edge_ids_taken)));

				//graph.vertex_map.at(vertex_ids_taken).get()->coord = Point(0,0,0);

				graph.vertex_map.at(vertex_ids_taken).get()->coord = coord;

				vertex_ids_taken++;
				edge_ids_taken++;

				if (!graph.vertex_map.contains(vertex_ids_taken))
				{
					vertex_ids_taken = -1;
				}
			}
		}
	}

	//push vector to adjacency_map
	graph.adjacency_map.insert(std::make_pair(vertex_id, adjacencies));
}



template <typename T_Graph>
bool construct_graph(T_Graph& graph, int radius)
{
	//determine vertex_number
	int vertex_number = radius * radius * radius;
	int vertex_max_id = vertex_number - 1;

	//building all necessary vertexs
	int id_count = 0;
	for (int i = 0; i < radius; i++)
	{
		for (int j = 0; j < radius; j++)
		{
			for (int k = 0; k < radius; k++)
			{
				graph.vertex_map.insert(std::make_pair(id_count, std::make_shared<Vertex>(id_count)));
				id_count++;
			}
		}
	}

	//counter for edge and vertex ids. vertex_ids could be drawn out of the vertex_map but counter is faster than use a ordered_map
	//iterating through the vertexs in a order or searching it every-time.
	int edge_ids_taken = 0;
	int vertex_ids_taken = 1;

	for (int i = 0; i < graph.vertex_map.size(); i++)
	{
		map_vertices(graph, i, edge_ids_taken, vertex_ids_taken);
	}

	std::cout << "vertex cout: " << graph.vertex_map.size() << std::endl;
	std::cout << "edge cout: " << graph.edge_map.size() << std::endl;

	return true;
}


//fill vector for changes each iteration
template <typename U_changes>
void change_environment(U_changes& changes)
{
	//one iteration of changes is a vector of pairs of node ids and the value, the edge_weights change to
	std::vector< std::vector<std::pair<int, double>> > global_changes;


	std::vector<std::pair<int, double>> c1;
	c1.push_back(std::make_pair(2, 5));
	c1.push_back(std::make_pair(3, 5));
	c1.push_back(std::make_pair(4, 5));
	c1.push_back(std::make_pair(5, 5));
	c1.push_back(std::make_pair(6, 5));
	c1.push_back(std::make_pair(7, 5));
	c1.push_back(std::make_pair(8, 10));
	c1.push_back(std::make_pair(9, 5));
	global_changes.push_back(c1);



	std::vector<std::pair<int, double>> c2;
	c2.push_back(std::make_pair(2, 5));
	c2.push_back(std::make_pair(3, 8));
	c2.push_back(std::make_pair(4, 8));
	c2.push_back(std::make_pair(5, 1));
	c2.push_back(std::make_pair(6, 4));
	c2.push_back(std::make_pair(7, 8));
	c2.push_back(std::make_pair(8, 1));
	c2.push_back(std::make_pair(9, 8));
	global_changes.push_back(c2);



	std::vector<std::pair<int, double>> c3;
	c3.push_back(std::make_pair(2, 2));
	c3.push_back(std::make_pair(3, 2));
	c3.push_back(std::make_pair(4, 2));
	c3.push_back(std::make_pair(5, 2));
	c3.push_back(std::make_pair(6, 2));
	c3.push_back(std::make_pair(7, 2));
	c3.push_back(std::make_pair(8, 2));
	c3.push_back(std::make_pair(9, 2));
	global_changes.push_back(c3);


	changes = global_changes;
}