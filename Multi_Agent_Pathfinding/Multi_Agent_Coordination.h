#pragma once

#include "Multi_Agent_Pathfinding.h"

template <typename T_graph>
bool is_occupied(T_graph& graph, const int& node_id)
{
	for (const auto& occ : graph.occupancy)
	{
		if (occ.second.first == node_id)
			return true;
	}
	return false;
}

template <typename T_graph, typename U_agent>
void agent_heuristic(const T_graph& graph, U_agent& agent, int dst_id)
{
	double h;
	calc_heuristic(graph, agent.get()->vertex_position, dst_id, 1, h);
	agent.get()->priority = h;
}

// only moving the agent one step
template <typename T_agent>

/*
void move_agent(T_agent& agent)
{
	//std::cout << " moving " << std::endl;
	//std::cout << " size: " << agent.get()->path.size() << std::endl;
	agent.get()->vertex_position = agent.get()->path.front();
	agent.get()->path.pop();
}
*/
void move_agent(T_agent& agent, const int& move_to)
{
	agent.get()->vertex_position = move_to;
}



/*
Every Agent holds a queue with the calculated path of D* Lite.
Instead of following the complete Path, we move all agents
to their next step.
Following this, the agents have to be inserted back into
the priority queue for the agents
*/
template <typename T_graph, typename U_agents, typename V_finished>
bool update_agents(T_graph& graph, U_agents& agents, V_finished& finished)
{
	for (auto& agent : agents)
	{

		std::cout << " path size og: " << agent.get()->id << " " << agent.get()->path.size() << std::endl;

		if (!finished.contains(agent.get()->id))
		{
			move_agent(agent, agent.get()->next_step);
			//move_agent(agent);

			if (agent.get()->vertex_position == graph.destinations.at(agent.get()->id))
			{
				finished.insert(agent.get()->id);
			}
			//std::cout << " move agent nr. " << agent.get()->id << " to: " << agent.get()->next_step << std::endl;
		}
	}
	//std::cout << std::endl;
	return true;
}



template <typename T_graph>
bool set_occupancies(T_graph& graph, double& time , int& vertex, int& obstacle_id)
{
	graph.occupancies.insert(std::make_pair(time, std::make_pair(vertex, obstacle_id)));
	return true;
}

// update_obstacles
template <typename T_graph>
bool update_obstacles(T_graph& graph)
{
	bool result = false;

	//TODO: Adding new obstacles after start

	//for all 0 < time < INT_MAX
	for (auto& occ : graph.occupancies)
	{
		if (occ.first > 0 && occ.first < INT_MAX)
		{
			auto node = graph.occupancies.extract(occ.first);
			node.key() -= constants::time_step;
			graph.occupancies.insert(std::move(node));
		}

		if (occ.first <= 0)
		{	
			for (auto& edg_ver : graph.adjacency_map.at(occ.second.first))
			{
				graph.edge_map.at(edg_ver.first).get()->edge_weight = 1;
			}
	
			result = true;
		}

		if (occ.first == INT_MAX)
		{
			for (auto& edg_ver : graph.adjacency_map.at(occ.second.first))
			{
				graph.edge_map.at(edg_ver.first).get()->edge_weight = INT_MAX;
			}
		}
	}


	auto iter = graph.occupancies.begin();
	auto endIter = graph.occupancies.end();

	for (; iter != endIter;)
	{
		if (iter->first <= 0)
		{
			iter = graph.occupancies.erase(iter);
		}
		else
		{
			++iter;
		}
	}

	return result;
}


template <typename T_graph, typename V_vertex_set>
void get_affecting_vertices(T_graph& graph, V_vertex_set& set, int& agent_id, double& local_time)
{
	auto range = graph.agent_occupancies.equal_range(local_time);

	for (auto it = range.first; it != range.second; ++it)
	{
		if(it->second.second != agent_id)
			set.insert(it->second.first);
	}
}


template <typename T_graph>
void initialize_obstacles(T_graph& graph)
{
	graph.occupancies.insert(std::make_pair(5000 , std::make_pair(16, 0)));
	graph.occupancies.insert(std::make_pair(5000, std::make_pair(17, 0)));
	graph.occupancies.insert(std::make_pair(5000, std::make_pair(18, 0)));
	graph.occupancies.insert(std::make_pair(2000, std::make_pair(101, 1)));
	graph.occupancies.insert(std::make_pair(2000, std::make_pair(102, 1)));
	graph.occupancies.insert(std::make_pair(INT_MAX, std::make_pair(408, 3)));
}

//Initialize
/*
We initialize the agents for the algorithm, setting their
start positions, ids and destinations.
They get pushed into the agents list and prio queue
*/
template <typename T_graph, typename U_list>
bool initialize_multiagent(T_graph& graph, U_list& agents)
{
	std::shared_ptr<Agent> a0 = std::make_shared<Agent>(0, 0);
	graph.destinations.insert(std::make_pair(0, 200));
	
	std::shared_ptr<Agent> a1 = std::make_shared<Agent>(1, 100);
	graph.destinations.insert(std::make_pair(1, 999));
	
	std::shared_ptr<Agent> a2 = std::make_shared<Agent>(2, 10);
	graph.destinations.insert(std::make_pair(2, 20));
	
	std::shared_ptr<Agent> a3 = std::make_shared<Agent>(3, 500);
	graph.destinations.insert(std::make_pair(3, 700));
	
	std::shared_ptr<Agent> a4 = std::make_shared<Agent>(4, 800);
	graph.destinations.insert(std::make_pair(4, 5));
	
	std::shared_ptr<Agent> a5 = std::make_shared<Agent>(5, 300);
	graph.destinations.insert(std::make_pair(5, 212));
	
	std::shared_ptr<Agent> a6 = std::make_shared<Agent>(6, 900);
	graph.destinations.insert(std::make_pair(6, 21));

	std::shared_ptr<Agent> a7 = std::make_shared<Agent>(7, 100);
	graph.destinations.insert(std::make_pair(7, 666));

	std::shared_ptr<Agent> a8 = std::make_shared<Agent>(8, 64);
	graph.destinations.insert(std::make_pair(8, 850));

	std::shared_ptr<Agent> a9 = std::make_shared<Agent>(9, 990);
	graph.destinations.insert(std::make_pair(9, 111));
	
	std::shared_ptr<Agent> a10 = std::make_shared<Agent>(10, 400);
	graph.destinations.insert(std::make_pair(10, 611));

	std::shared_ptr<Agent> a11 = std::make_shared<Agent>(11, 350);
	graph.destinations.insert(std::make_pair(11, 735));

	std::shared_ptr<Agent> a12 = std::make_shared<Agent>(12, 864);
	graph.destinations.insert(std::make_pair(12, 425));

	std::shared_ptr<Agent> a13 = std::make_shared<Agent>(13, 190);
	graph.destinations.insert(std::make_pair(13, 698));

	std::shared_ptr<Agent> a14 = std::make_shared<Agent>(14, 343);
	graph.destinations.insert(std::make_pair(14, 117));
	

	
	agent_heuristic(graph, a0, 200);
	agents.insert(a0);
	
	agent_heuristic(graph, a1, 999);
	agents.insert(a1);
	
	agent_heuristic(graph, a2, 20);
	agents.insert(a2);

	agent_heuristic(graph, a3, 700);
	agents.insert(a3);

	agent_heuristic(graph, a4, 5);
	agents.insert(a4);

	agent_heuristic(graph, a5, 212);
	agents.insert(a5);

	agent_heuristic(graph, a6, 21);
	agents.insert(a6);

	agent_heuristic(graph, a7, 666);
	agents.insert(a7);

	agent_heuristic(graph, a8, 850);
	agents.insert(a8);

	agent_heuristic(graph, a9, 111);
	agents.insert(a9);

	agent_heuristic(graph, a10, 611);
	agents.insert(a10);

	agent_heuristic(graph, a11, 725);
	agents.insert(a11);

	agent_heuristic(graph, a12, 425);
	agents.insert(a12);
	
	agent_heuristic(graph, a13, 698);
	agents.insert(a13);

	agent_heuristic(graph, a14, 117);
	agents.insert(a14);
	

	//init every start and destination as infinity

	//every start
	for (auto& agent : agents)
	{
		for (auto& edg_ver : graph.adjacency_map.at(agent.get()->vertex_position))
		{
			graph.edge_map.at(edg_ver.first).get()->edge_weight = INT_MAX;
		}
	}

	//every destination
	for (auto& agt_dst : graph.destinations)
	{
		for (auto& edg_ver : graph.adjacency_map.at(agt_dst.second))
		{
			graph.edge_map.at(edg_ver.first).get()->edge_weight = INT_MAX;
		}
	}


	//	TODO: initialize Obstacles

	return true;
}

/*
destinations->obstacles->paths

*/
template <typename T_graph>
void set_edges(T_graph& graph, const int& vertex_id, const double edge_weight, bool priority )
{

	//only set edges to vertices which are no goal/start/agent obstacle or global obstacle.
	//does only matter if edges are blocked

	//destinations have the highest priority

	if (priority)
	{
		for (auto& edg_ver : graph.adjacency_map.at(vertex_id))
		{
			graph.edge_map.at(edg_ver.first).get()->edge_weight = edge_weight;
		}
	}
	else
	{
		for (auto& edg_ver : graph.adjacency_map.at(vertex_id))
		{
			if (!graph.is_goal(edg_ver.second) && !graph.is_occupancy(edg_ver.second))
			//if(!graph.is_occupancy(edg_ver.second) )
			{
				if (graph.is_agent_occupancy(edg_ver.second))
				{
					double own_time;
					double target_time;

					for (const auto& [time, ver_agt] : graph.agent_occupancies)	//iterator for time range
					{
						if (ver_agt.first == edg_ver.second)
							target_time = time;
						
						if(ver_agt.first == vertex_id)
							own_time = time;
					}

					if(own_time != target_time)
						graph.edge_map.at(edg_ver.first).get()->edge_weight = edge_weight;
				}
				else 
				{
					graph.edge_map.at(edg_ver.first).get()->edge_weight = edge_weight;
				}
			}
			else
			{
				graph.edge_map.at(edg_ver.first).get()->edge_weight = edge_weight;
			}
		}
	}


	//"reset" g and rhs
	graph.vertex_map.at(vertex_id).get()->cost_g = INT_MAX;
	graph.vertex_map.at(vertex_id).get()->cost_rhs = INT_MAX;
}

// multirobot d* lite
/*
We dont use signals to notify the algorithm of any changes in the graph
Instead, we manage a list with all nodes influenced by timed obstacles
i.e paths of agents with a higher priority, dynamic obstacles, etc..
The graph is adapted before every call to dstarlite_main, which then
calculates the complete path the specific agent has to follow.
*/
template <typename T_graph, typename U_agents, typename V_finished>
bool multiagent_dlite(T_graph& graph, U_agents& agents, V_finished& finished)
{
	for (auto& agent : agents)
	{
		if (!finished.contains(agent.get()->id))
		{
			//set start and goal only for own agent to 1.
			//free own position and dst
			int dst_id = 0;
			get_dst(graph, agent, dst_id);

			set_edges(graph, agent.get()->vertex_position, 1, false);
			set_edges(graph, dst_id, 1, true);

			//used to reset all planned vertices
			std::set<int> path;
	
			agent.get()->path = {};

			dstarlite_main(graph, agent, path);

			//reset start and goal
			set_edges(graph, agent.get()->vertex_position, INT_MAX, false);
			set_edges(graph, dst_id, INT_MAX, true);
			
			//resetting all calculated costs of all vertices of the path. 
			//the next agent must not use calculated values from other paths
			for (auto& vertex_id : path)
			{
				graph.vertex_map.at(vertex_id).get()->cost_g = INT_MAX;
				graph.vertex_map.at(vertex_id).get()->cost_rhs = INT_MAX;
			}

			for (auto& [vertex_id, shr_vertex] : graph.vertex_map)
			{
				if (shr_vertex.get()->cost_g != INT_MAX)
					shr_vertex.get()->cost_g = INT_MAX;

				if (shr_vertex.get()->cost_rhs != INT_MAX)
					shr_vertex.get()->cost_rhs = INT_MAX;
			}
		}
	}
	return true;
}



//	Multi - Agent Main
/*
initialize();
forever
	multirobot_D*Lite(robot_list, G)
	update_robot_list(robot_list, G)
	update(S);	//update start positions to current positions
				//a req of D* Lite
*/
template <typename T_graph, typename U_agents>
bool multiagent_main(T_graph& graph, U_agents& agents, int& iterations)
{
	std::set<int> finished;
	
	bool changed_detected = true;

	while (finished.size() != agents.size())
	{
		//path calculation

		//if (changed_detected)
		//{
			//std::cout << "changes detected " << std::endl;

			multiagent_dlite(graph, agents, finished);

			/*
			std::cout << std::endl;
			std::cout << " all agent occupancies " << std::endl;
			for (const auto& [key, ver_agt] : graph.agent_occupancies)
			{
				std::cout << "time: " << key << " vertex: " << ver_agt.first << " agent: " << ver_agt.second << std::endl;
			}
			*/

			//clear all agent timings,
			graph.agent_occupancies.clear();

		//}
		
		//std::cout << " updating " << std::endl;

		// update agents
		update_agents(graph, agents, finished);
		
		//call multiagent for every agent until all finished \newl
		update_obstacles(graph);


		iterations++;
	}
	return true;
}