/*
MIT License

Copyright(c)[2022][Dennis Wilpert]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this softwareand associated documentation files(the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and /or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions :

The above copyright noticeand this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "Multi_Agent_Coordination.h"



int main()
{
	Graph graph;
	construct_graph(graph, 10);

	auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
	std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

	initialize_multiagent(graph, agents);
	initialize_obstacles(graph);

	int iterations = 0;
	multiagent_main(graph, agents, iterations);

	//dstarlite_main(graph, 0, 200);
	
	return 0;
}


//initialize changing iterations
	//std::vector< std::vector<std::pair<int, double>> > global_changes;
		//change_environment(global_changes);

	//========== initialize ==========
	// OPEN = EMPTY
	// k_m = 0;
	// all nodes rhs = g = INFINITY. Before a node is in any way checked, set rhs and g to Infinity
	//rhs s_goal = 0
	// OPEN.Insert s_goal, Calckey)



/*
for (auto& ver : blocking_vertices)
{
	// Update the edge cost c(u,v)
	set_edges(graph, ver, 1);

	//update vertex (u)
	update_dstar_vertex(graph, open_queue, open_tracker, ver, src_id, dst_id, k_m);
}
*/

/*
for (auto& ver : blocking_vertices)
{
	// Update the edge cost c(u,v)
	set_edges(graph, ver, INT_MAX);

	//update vertex (u)
	update_dstar_vertex(graph, open_queue, open_tracker, ver, src_id, dst_id, k_m);
}
*/
/*
//for all edges(u,v) with changed edge costs
for (auto& node_weight : global_changes.at(change_timings))
{
	for (auto& edg_ver : graph.adjacency_map.at(node_weight.first))
	{
		// Update the edge cost c(u,v)
		graph.edge_map.at(edg_ver.first).get()->edge_weight = node_weight.second;

		//update_dstar_vertex(graph, open_queue, open_tracker, edg_ver.second, src_id, dst_id, k_m);
	}
	//update vertex (u)

}
*/