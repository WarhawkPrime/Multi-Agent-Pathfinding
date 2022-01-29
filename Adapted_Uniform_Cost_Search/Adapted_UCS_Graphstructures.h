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
	Vertex(int id) : id{ id }
	{
		this->cost_g = 1;
		this->cost_h = 0;
		this->cost_f = cost_g + cost_h;
		this->coord = Point(0, 0, 0);
	};
	Vertex(int id, double cost_g) : id{ id }, cost_g{ cost_g }
	{
		this->cost_h = 0;
		this->cost_f = cost_g + cost_h;
		this->coord = Point(0, 0, 0);
	}
	Vertex(int id, double cost_g, Point coord) : id{ id }, cost_g{ cost_g }, coord{ coord }
	{
		this->cost_h = 0;
		this->cost_f = cost_g + cost_h;
	}
	Vertex(int id, Point coord) : id{ id }, cost_g{ cost_g }, coord{ coord }
	{
		this->cost_g = 1;
		this->cost_h = 0;
		this->cost_f = cost_g + cost_h;
	}

	int id;

	double cost_g = 0;
	double cost_h = 0;
	double cost_f = 0;

	Point coord;

	void calc_f(double g, double h) { this->cost_f = g + h; }
};


struct Edge
{
	Edge() {};
	Edge(int id) : id{ id } { this->edge_weight = 1; };
	Edge(int id, double edge_weight) : id{ id }, edge_weight{ edge_weight } {};
	int id;
	double edge_weight;
};

struct Compare_Nodes
{
	bool operator()(std::shared_ptr<Vertex> n1, std::shared_ptr<Vertex> n2)
	{
		return n1.get()->cost_g > n2.get()->cost_g;
	}
};

class Graph
{
public:
	Graph() {};

	//map with all nodes, the key is the id.
	std::unordered_map<int, std::shared_ptr<Vertex>> vertex_map;

	//map with all edges, the key is the id
	std::unordered_map<int, std::shared_ptr<Edge>> edge_map;

	//set to map two nodes with an edge. first is id for the node, second is vector of (edge_id, node_id). vector can be used to get all adj. and iterate through them
	std::unordered_map<int, std::vector<std::pair<int, int>> > adjacency_map;

	//getter for nodes and edges
	std::shared_ptr<Vertex> get_node(int node_id) { return vertex_map.at(node_id); }
	std::shared_ptr<Edge> get_edge(int edge_id) { return edge_map.at(edge_id); }
	std::vector<std::pair<int, int>> get_adjacencies(int node_id) { return adjacency_map.at(node_id); }

	//getter for coordinates
	double get_x_coord(int node_id) const { return std::get<0>(vertex_map.at(node_id).get()->coord); }
	double get_y_coord(int node_id) const { return std::get<1>(vertex_map.at(node_id).get()->coord); }
	double get_z_coord(int node_id) const { return std::get<2>(vertex_map.at(node_id).get()->coord); }

	//getter setter for values
	double get_g_costs(int node_id) const { return vertex_map.at(node_id).get()->cost_g; }
	void set_g_cost(int node_id, double new_cost_g) { vertex_map.at(node_id).get()->cost_g = new_cost_g; }

	//inserts
	void add_node(int id, std::shared_ptr<Vertex> node) { vertex_map.insert(std::make_pair(id, node)); }
	void add_edge(int id, std::shared_ptr<Edge> edge) { edge_map.insert(std::make_pair(id, edge)); }
	void add_adjacencies(int id, std::vector<std::pair<int, int>> adj) { adjacency_map.insert(std::make_pair(id, adj)); }


private:

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
void map_nodes(T_Graph& graph, unsigned int& node_id, int& edge_ids_taken, int& node_ids_taken)
{
	//create adjacencie vector holding all adjacencies for the node node_id
	std::vector<std::pair<int, int>> adjacencies;

	//sets to hold all possible neighbours of node_id (26)
	std::set<Point> possible_directions;

	// if id is 0, its the first node. all other nodes are build around it and so already have coordinates
	if (node_id == 0)
	{
		graph.vertex_map.at(node_id).get()->coord = Point(0, 0, 0);
	}

	//get all possible nodes current node can achieve
	build_directions(possible_directions, graph.vertex_map.at(node_id).get()->coord);

	//for every (node_id, vector<edge_id, neighbour_id>), search if it lists current as an neighbour
	for (const auto& elem : graph.adjacency_map)
	{
		//vector of edge_id, neighbour_id
		bool neigh = false;
		for (auto& ed_ne : elem.second)
		{
			//current node is mentioned as neighbour
			if (ed_ne.second == node_id)
			{
				//so elem is a neighbour of current with an existing edge
				adjacencies.push_back(std::make_pair(ed_ne.first, elem.first));
				neigh = true;
			}
		}
	}


	//store node_ids into a set. Use another set to store id that have to be added after the loop
	std::set<int> adj_nodes;
	std::set<int> to_add;

	//fill adj set
	for (const auto& elem : adjacencies)
	{
		adj_nodes.insert(elem.second);
	}

	//for all neighbours in adjacencies vector
	for (const auto& elem : adjacencies)
	{
		//for all edge_nodeId pairs in the corresponding adjacency list
		for (auto& ed_ne : graph.adjacency_map.at(elem.second))
		{
			//check if direction is possible
			if (possible_directions.count(graph.vertex_map.at(ed_ne.second).get()->coord))
			{
				//check if id isn't already in the adj list (with the help of the set)
				if (!adj_nodes.count(ed_ne.second))
				{
					//insert node id to set if no duplicate
					to_add.insert(ed_ne.second);
				}
			}
		}
	}

	//now add the nodes to the adjacency vector. create edges
	for (const auto& id : to_add)
	{
		adjacencies.push_back(std::make_pair(edge_ids_taken, id));
		graph.edge_map.insert(std::make_pair(edge_ids_taken, std::make_shared<Edge>(edge_ids_taken)));
		edge_ids_taken++;
	}

	//now, see if we need to add left nodes to current node.
	//substract adj from possible dirs, the rest are new neighbours. If! they are never before
	//mentioned as a neighbour.
	for (const auto& coord : adjacencies)
	{
		possible_directions.erase(graph.vertex_map.at(coord.second).get()->coord);
	}

	//if node doesnt exist, set -1
	if (!graph.vertex_map.count(node_ids_taken))
	{
		node_ids_taken = -1;
	}

	//node_ids_taken means that no free nodes exist
	if (node_ids_taken != -1)
	{
		//for every possible direction, add free node in adj vector, cac its coords
		for (const auto& coord : possible_directions)
		{
			if (node_ids_taken != -1)
			{
				//push node with new edge, increase counters, set coordinate of node
				adjacencies.push_back(std::make_pair(edge_ids_taken, node_ids_taken));
				graph.edge_map.insert(std::make_pair(edge_ids_taken, std::make_shared<Edge>(edge_ids_taken)));

				//graph.vertex_map.at(node_ids_taken).get()->coord = Point(0,0,0);

				graph.vertex_map.at(node_ids_taken).get()->coord = coord;

				node_ids_taken++;
				edge_ids_taken++;

				if (!graph.vertex_map.count(node_ids_taken))
				{
					node_ids_taken = -1;
				}
			}
		}
	}

	//push vector to adjacency_map
	graph.adjacency_map.insert(std::make_pair(node_id, adjacencies));
}



template <typename T_Graph>
bool construct_graph(T_Graph& graph, const int& radius)
{
	//determine node_number
	int node_number = radius * radius * radius;
	int node_max_id = node_number - 1;

	//building all necessary nodes
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

	//counter for edge and node ids. node_ids could be drawn out of the vertex_map but counter is faster than use a ordered_map
	//iterating through the nodes in a order or searching it every-time.
	int edge_ids_taken = 0;
	int node_ids_taken = 1;

	for (unsigned int i = 0; i < graph.vertex_map.size(); i++)
	{
		map_nodes(graph, i, edge_ids_taken, node_ids_taken);
	}

	std::cout << " node cout: " << graph.vertex_map.size() << std::endl;
	std::cout << " edge cout: " << graph.edge_map.size() << std::endl;

	return true;
}