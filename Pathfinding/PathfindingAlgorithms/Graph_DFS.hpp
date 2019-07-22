#pragma once

/**
*	Graph_DFS.hpp
*
*	Template class to execute the Depth First Search algorithm
*
*/

#include "RTSCommon/AISystem/GraphSystem/Graph.hpp"
#include <stack>


template <class graph_type>
class Graph_DFS
{

private:

	enum {VISITED, UNVISITED, NO_PARENT};
	typedef typename graph_type::EdgeType Edge;

public:

	Graph_DFS(graph_type& graph, int source, int target = -1, bool iterative = true)
		: graph(graph), sourceNode(source), targetNode(target), targetNodeFound(false),
		visitedNodeVector(graph.GetNumNodes(), UNVISITED), routeToTargetVector(graph.GetNumNodes(), NO_PARENT)
	{
		if (iterative)
		{
			targetNodeFound = IterativeSearch();
		}
		else
		{
			targetNodeFound = RecursiveSearch();
		}

	}

	virtual ~Graph_DFS() {}

	bool IsTargetFound() const { return targetNodeFound; }

	///the function returns the path from source to target nodeIndex moving backwards (from target to source). So first element is the target while the last one is the source</summary
	std::list<int> GetPathToTarget() const;

private:

	bool IterativeSearch();
	bool RecursiveSearch(const Edge* edge = NULL);
	
	//reference to the graph to search
	const graph_type &graph;
	//list of visited nodes (indices). it will have store a number of elements equal to the number of nodes, however, that elements will be equal to specific value to declare if a nodeIndex has been visited or not
	std::vector<int> visitedNodeVector;
	//list of nodes leading from the source to the target nodeIndex. It will be read starting from the target nodeIndex. It will be set in the same wasy as the vector of visited nodes
	std::vector<int> routeToTargetVector;

	//source and target nodeIndex
	int sourceNode, targetNode;
	bool targetNodeFound;

};


template<class graph_type>
bool Graph_DFS<graph_type>::IterativeSearch()
{
	///<summary>Stack of edges pointing to unvisited nodes</summary>
	//use of stack because LIFO
	std::stack<const Edge*> EdgeStack;
	//dummy edge to start the while loop. In this way, the algorithm will mark the source nodeIndex as visited
	Edge edge(sourceNode, sourceNode);
	EdgeStack.push(&edge);

	//until the stack of edges is not empty
	while (!EdgeStack.empty())
	{
		//get last edge and remove it from the stack
		const Edge* nextEdge = EdgeStack.top();
		EdgeStack.pop();

		//set as visisted and the parent nodeIndex of the nodeIndex the edge is pointing
		routeToTargetVector[nextEdge->ToNode()] = nextEdge->FromNode();
		visitedNodeVector[nextEdge->ToNode()] = VISITED;

		//if pointed nodeIndex is the target
		if (nextEdge->ToNode() == targetNode)
		{
			return true;
		}

		//instance of the edge iterator to iterate through all the edges of a nodeIndex
		graph_type::EdgeIterator edgeItr(graph, nextEdge->ToNode());
		for (const Edge* iEdge = edgeItr.Begin(); !edgeItr.End(); iEdge = edgeItr.NextEdge())
		{
			if (visitedNodeVector[iEdge->ToNode()] == UNVISITED)
			{
				EdgeStack.push(iEdge);
			}
		}
	}

	return false;

}


template<class graph_type>
bool Graph_DFS<graph_type>::RecursiveSearch(const Edge *pEdge)
{
	if (pEdge == NULL)
	{
		const Edge edge(sourceNode, sourceNode);
		RecursiveSearch(&edge);
	}
	else
	{
		routeToTargetVector[pEdge->ToNode()] = pEdge->FromNode();
		visitedNodeVector[pEdge->ToNode()] = VISITED;

		//if pointed nodeIndex is the target
		if (pEdge->ToNode() == targetNode)
		{
			return true;
		}

		graph_type::EdgeIterator edgeItr(graph, pEdge->ToNode());
		for (const Edge* edge = edgeItr.Begin(); !edgeItr.End(); edge = edgeItr.NextEdge())
		{
			if (visitedNodeVector[edge->ToNode()] == UNVISITED)
			{
				RecursiveSearch(edge);
			}
		}
	}

	return false;

}


///the function returns the path from source to target nodeIndex moving backwards (from target to source). So first element is the target while the last one is the source
template<class graph_type>
std::list<int> Graph_DFS<graph_type>::GetPathToTarget() const
{
	std::list<int> pathToTarget;

	//in case of target not targetNodeFound or target is an invalid nodeIndex
	if (!targetNodeFound || targetNode < 0)
	{
		return pathToTarget;
	}

	pathToTarget.push_back(targetNode);
	int node = targetNode;

	//store nodes from target to source
	while (node != sourceNode)
	{
		pathToTarget.push_back(routeToTargetVector[node]);
		//retrive the parent nodeIndex
		node = routeToTargetVector[node];
	}

	return pathToTarget;

}
