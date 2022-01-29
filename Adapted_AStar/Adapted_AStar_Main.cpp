

#include "Adapted_AStar.h"

int main()
{
	Graph environment;
	construct_graph(environment, 3);
	astar_hotqueue(environment);
	
	return 0;
}