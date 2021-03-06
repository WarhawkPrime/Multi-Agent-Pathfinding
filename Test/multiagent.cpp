

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

#include <chrono>

#include "Multi_Agent_Pathfinding/Multi_Agent_Coordination.h"

TEST_SUITE("Multi-Agent-Pathfinding tests" *doctest::description("functional tests"))
{

	TEST_CASE("functional tests" )
	{
		SUBCASE("Graph Generation" )
		{
			int size = 3;

			Graph graph;
			bool res = construct_graph(graph, size);

			CHECK(res == true);

			size_t ver_count = graph.vertex_map.size();
			size_t edge_count = graph.edge_map.size();

			int dimensions = size * size * size;

			CHECK(ver_count == dimensions);

			CHECK(edge_count <= (dimensions * 26));
		}
		SUBCASE("Single Agent No Obstacles Small Graph")
		{
			Graph graph;
			construct_graph(graph, 3);

			auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
			std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

			std::shared_ptr<Agent> a0 = std::make_shared<Agent>(0, 1);
			graph.destinations.insert(std::make_pair(0, 8));

			agent_heuristic(graph, a0, 8);
			agents.insert(a0);

			int iterations = 0;
			auto start = std::chrono::high_resolution_clock::now();
			bool res = multiagent_main(graph, agents, iterations);
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

			CHECK(res);

			CHECK(duration.count() < 1000000);
			std::cout << " SASG: " << duration << std::endl;
		}
		SUBCASE("Single Agent No Obstacles Large Graph")
		{
			Graph graph;
			construct_graph(graph, 10);

			auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
			std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

			std::shared_ptr<Agent> a0 = std::make_shared<Agent>(0, 100);
			graph.destinations.insert(std::make_pair(0, 999));

			agent_heuristic(graph, a0, 999);
			agents.insert(a0);

			int iterations = 0;
			auto start = std::chrono::high_resolution_clock::now();
			bool res = multiagent_main(graph, agents, iterations);
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

			CHECK(res);

			CHECK(duration.count() < 1000000);
			std::cout << "SALG: " << duration << std::endl;
		}
		SUBCASE("Multi Agent No Obstacles Small Graph")
		{
			Graph graph;
			construct_graph(graph, 3);

			auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
			std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

			std::shared_ptr<Agent> a0 = std::make_shared<Agent>(0, 1);
			graph.destinations.insert(std::make_pair(0, 8));

			agent_heuristic(graph, a0, 8);
			agents.insert(a0);

			std::shared_ptr<Agent> a1 = std::make_shared<Agent>(1, 3);
			graph.destinations.insert(std::make_pair(1, 6));

			agent_heuristic(graph, a1, 6);
			agents.insert(a1);
	
			std::shared_ptr<Agent> a2 = std::make_shared<Agent>(2, 4);
			graph.destinations.insert(std::make_pair(2, 7));

			agent_heuristic(graph, a2, 7);
			agents.insert(a2);

			std::shared_ptr<Agent> a3 = std::make_shared<Agent>(3, 5);
			graph.destinations.insert(std::make_pair(3, 2));

			agent_heuristic(graph, a3, 2);
			agents.insert(a3);
		
			int iterations = 0;
			auto start = std::chrono::high_resolution_clock::now();
			bool res = multiagent_main(graph, agents, iterations);
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

			CHECK(res);

			CHECK(duration.count() < 1000000);
			std::cout << "MASG: " << duration << std::endl;
		}
		SUBCASE("Multi Agent No Obstacles Large Graph")
		{
			Graph graph;
			construct_graph(graph, 10);

			auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
			std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

			initialize_multiagent(graph, agents);
			int iterations = 0;

			auto start = std::chrono::high_resolution_clock::now();
			bool res = multiagent_main(graph, agents, iterations);
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

			CHECK(res);

			CHECK(duration.count() < 1000000);
			std::cout << "MALG: " << duration << std::endl;
		}
	}
}



int single_agent(int src, int dst,std::chrono::microseconds& dur)
{
	Graph graph;
	construct_graph(graph, 3);

	auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
	std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

	std::shared_ptr<Agent> a0 = std::make_shared<Agent>(0, src);
	graph.destinations.insert(std::make_pair(0, dst));

	agent_heuristic(graph, a0, dst);
	agents.insert(a0);

	int iterations = 0;

	auto start = std::chrono::high_resolution_clock::now();
	bool res = multiagent_main(graph, agents, iterations);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	dur = duration;

	return iterations;
}


int single_agent_large_graph(int src, int dst, std::chrono::microseconds& dur)
{
	Graph graph;
	construct_graph(graph, 10);

	auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
	std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

	std::shared_ptr<Agent> a0 = std::make_shared<Agent>(0, src);
	graph.destinations.insert(std::make_pair(0, dst));

	agent_heuristic(graph, a0, dst);
	agents.insert(a0);

	int iterations = 0;

	auto start = std::chrono::high_resolution_clock::now();
	bool res = multiagent_main(graph, agents, iterations);
	auto stop = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

	dur = duration;

	return iterations;
}




TEST_SUITE("Multi-Agent-Pathfinding tests" * doctest::description("non-functional tests"))
{
	TEST_CASE("non-functional tests")
	{
		SUBCASE("small graph multi or single")
		{

			// 4 single angents
			std::chrono::microseconds first;
			int res1 = single_agent(1, 8, first);

			std::chrono::microseconds second;
			int res2 = single_agent(3, 6, second);

			std::chrono::microseconds third;
			int res3 = single_agent(4, 7, third);

			std::chrono::microseconds fourth;
			int res4 = single_agent(5, 2, fourth);

			


			// 4 agents coordinated
			Graph graph;
			construct_graph(graph, 3);

			auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
			std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

			std::shared_ptr<Agent> a0 = std::make_shared<Agent>(0, 1);
			graph.destinations.insert(std::make_pair(0, 8));

			agent_heuristic(graph, a0, 8);
			agents.insert(a0);

			std::shared_ptr<Agent> a1 = std::make_shared<Agent>(1, 3);
			graph.destinations.insert(std::make_pair(1, 6));

			agent_heuristic(graph, a1, 6);
			agents.insert(a1);

			std::shared_ptr<Agent> a2 = std::make_shared<Agent>(2, 4);
			graph.destinations.insert(std::make_pair(2, 7));

			agent_heuristic(graph, a2, 7);
			agents.insert(a2);

			std::shared_ptr<Agent> a3 = std::make_shared<Agent>(3, 5);
			graph.destinations.insert(std::make_pair(3, 2));

			agent_heuristic(graph, a3, 2);
			agents.insert(a3);

			int iterations = 0;

			auto start = std::chrono::high_resolution_clock::now();
			bool res = multiagent_main(graph, agents, iterations);
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

			auto sum = first + second + third + fourth;
			auto s = sum.count();
			auto m = duration.count();
			std::cout << " sum: " << sum << std::endl;
			std::cout << " duration " << duration << std::endl;
			
			CHECK(m < s);

			int res_sum = res1 + res2 + res3 + res4;
			std::cout << "iter: " << iterations << std::endl;
			std::cout << "iter: " << res_sum << std::endl;
			CHECK(iterations < res_sum);

		}
		SUBCASE("large graph multi or single")
		{
			Graph graph;
			construct_graph(graph, 10);

			auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
			std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

			initialize_multiagent(graph, agents);

			int iterations = 0;

			auto start = std::chrono::high_resolution_clock::now();
			bool res = multiagent_main(graph, agents, iterations);
			auto stop = std::chrono::high_resolution_clock::now();
			auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);


			std::chrono::microseconds first;
			int res1 = single_agent_large_graph(0, 200, first);

			std::chrono::microseconds second;
			int res2 = single_agent_large_graph(100, 999, second);

			std::chrono::microseconds third;
			int res3 = single_agent_large_graph(10, 20, third);

			std::chrono::microseconds fourth;
			int res4 = single_agent_large_graph(500, 700, fourth);

			std::chrono::microseconds fith;
			int res5 = single_agent_large_graph(800, 5, fith);

			std::chrono::microseconds sixth;
			int res6 = single_agent_large_graph(300, 212, sixth);

			std::chrono::microseconds seventh;
			int res7 = single_agent_large_graph(900, 21, seventh);

			std::chrono::microseconds eighth;
			int res8 = single_agent_large_graph(100, 666, eighth);

			std::chrono::microseconds ninth;
			int res9 = single_agent_large_graph(64, 850, ninth);

			std::chrono::microseconds tenth;
			int res10 = single_agent_large_graph(999, 9, tenth);


			auto sum = first + second + third + 
						fourth + fith + sixth + 
						seventh + eighth + ninth +
						tenth ;

			auto res_sum = res1 + res2 + res3 + res4 + res5 + res6 + res7 + res8 + res9 + res10;

			//std::cout << "first" << first << std::endl;
			std::cout << "second" << second << std::endl;

			auto s = sum.count();
			auto m = duration.count();
			std::cout << " sum lg: " << sum << std::endl;
			std::cout << " duration lg" << duration << std::endl;
	
			CHECK(m > s);

			std::cout << "iter: " << iterations << std::endl;
			std::cout << "iter res_sum: " << res_sum << std::endl;

			CHECK(iterations < res_sum);
		}
		SUBCASE("timings")
		{

			std::chrono::microseconds sum;
			int count = 50;

			Graph graph; 
			construct_graph(graph, 10);

			for (int i = 0; i < count; i++)
			{
				auto agent_cmp = [](std::shared_ptr<Agent> a0, std::shared_ptr<Agent> a1) {return a0.get()->priority < a1.get()->priority; };
				std::set<std::shared_ptr<Agent>, decltype(agent_cmp)> agents;

				initialize_multiagent(graph, agents);
				initialize_obstacles(graph);


				int iterations = 0;

				auto start = std::chrono::high_resolution_clock::now();
				multiagent_main(graph, agents, iterations);
				auto stop = std::chrono::high_resolution_clock::now();
				auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

				sum += duration;

				auto d = duration.count();
				//std::cout << "duration: " << d << std::endl; 

			}

			auto s = sum.count();
			double median = s / count;
			
			std::cout << " median for 50: " << median << std::endl;

		}

	}
}
