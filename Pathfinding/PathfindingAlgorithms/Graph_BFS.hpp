#pragma once

#include "RTSCommon/AISystem/GraphSystem/Graph.hpp"
#include <ctime>


template <class graph_type>
class Graph_BFS
{

private:
	
	//to mark the nodes inside the vectors
	enum {VISITED, UNVISITED, NO_PARENT};
	typedef typename graph_type::EdgeType Edge;

public:

	Graph_BFS(graph_type& graph, int source, int target = -1)
		: graph(graph), sourceNode(source), targetNode(target), targetNodeFound(false),
		visitedNodeVector(graph.GetNumNodes(), UNVISITED), routeToTargetVector(graph.GetNumNodes(), NO_PARENT)
	{
		targetNodeFound = Search();

	}

	virtual ~Graph_BFS() {}

	bool IsTargetFound() const { return targetNodeFound; }
	std::list<int> GetPathToTarget() const;

private:

	bool Search();

	//const reference to the graph
	const graph_type& graph;
	int sourceNode, targetNode;
	bool targetNodeFound;

	//vectors to store visited nodes and the route to the target. they contains as much elements as nodes and their are set to UNVISITED and NO_PARENT
	std::vector<int> visitedNodeVector;
	std::vector<int> routeToTargetVector;

};


template<class graph_type>
bool Graph_BFS<graph_type>::Search()
{
	//use of queue because of FIFO
	std::list<const Edge*> EdgeQueue;
	//dummy edge
	const Edge dummyEdge(sourceNode, sourceNode);
	EdgeQueue.push_back(&dummyEdge);

	visitedNodeVector[sourceNode] = VISITED;

	while (!EdgeQueue.empty())
	{
		const Edge* nextEdge = EdgeQueue.front();
		EdgeQueue.pop_front();

		routeToTargetVector[nextEdge->ToNode()] = nextEdge->FromNode();

		if (nextEdge->ToNode() == targetNode)
		{
			return true;
		}

		graph_type::EdgeIterator edgeItr(graph, nextEdge->ToNode());
		for (const Edge* iEdge = edgeItr.Begin(); !edgeItr.End(); iEdge = edgeItr.NextEdge())
		{
			if (visitedNodeVector[iEdge->ToNode()] == UNVISITED)
			{
				//the nodeIndex is marked as visited here, before they are examinated, to avoid to store a large number of edges into the queue
				visitedNodeVector[iEdge->ToNode()] = VISITED;
				EdgeQueue.push_back(iEdge);
			}
		}
	}
	
	return false;

}


template<class graph_type>
std::list<int> Graph_BFS<graph_type>::GetPathToTarget() const
{
	std::list<int> path;

	if (!targetNodeFound || targetNode < 0)
	{
		return path;
	}

	int node = targetNode;
	path.push_back(node);

	while (node != sourceNode)
	{
		node = routeToTargetVector[node];
		path.push_back(node);
	}

	return path;

}
