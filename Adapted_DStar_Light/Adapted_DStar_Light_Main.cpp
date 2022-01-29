
#include "Adapted_DStar_Light.h"

int main()
{
	Graph graph;
	construct_graph(graph, 10);

	dstarlite_main(graph, 0, 200);

	return 1;
}