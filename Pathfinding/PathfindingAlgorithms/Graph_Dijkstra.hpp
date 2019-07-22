#pragma once

/**
*	Graph_Dijkstra.hpp
*
*	Template class to execute Dijkstra algorithm
*/

#include "RTSCommon/AISystem/GraphSystem/Graph.hpp"
#include "RTSCommon/AISystem/Utilities/PriorityQueue.hpp"
#include <queue>
#include <vector>


template <class graph_type>
class Dijkstra
{

private:

	enum {VISITED, UNVISITED, NO_PARENT};
	typedef typename graph_type::EdgeType Edge;
	typedef typename graph_type::NodeType Node;

public:

	Dijkstra(const graph_type& graph, int source, int target = -1) : graph(graph), sourceNode(source), targetNode(target),
		shortestPathTree(graph.GetNumNodes()), totalCostTo(graph.GetNumNodes()), frontier(graph.GetNumNodes())
	{
		targetNodeFound = Search();

	}

	~Dijkstra() {}

	std::vector<const Edge*> GetAllPaths() const { return shortestPathTree; }

	std::list<int> GetPathToTarget() const
	{
		std::list<int> path;
		if (!targetNodeFound)
		{
			return path;
		}

		int node = targetNode;
		path.push_back(node);

		while (node != sourceNode)
		{
			node = shortestPathTree[node]->FromNode();
			path.push_back(node);
		}

		return path;

	}

	double GetTotalCostToTarget() const { return totalCostTo[targetNode]; }

private:

	bool Search();

	const graph_type& graph;

	///<summary>holds the total cost of the best path from any nodeIndex to the source. The vector is indexed into by nodeIndex index.
	///example: totalCostTo[5] holds the total cost the edges of the best path targetNodeFound so far between nodeIndex 5 and the source nodeIndex</summary>
	std::vector<double> totalCostTo;

	///<summary>the adjacency edges leading to nodes not yet into the SPT. vector indexed into by nodeIndex index
	///it stores "parent" edges leading to nodes connected to the SPT but not inside of it yet</summary>
	std::vector<const Edge*> frontier;
	///<summary>the edges for the shortest path tree. it encapsulates the best path from any nodeIndex in the SPT to the source nodeIndex.
	///stores the edges of nodes of the SPT</summary>
	std::vector<const Edge*> shortestPathTree;

	int sourceNode, targetNode;

	bool targetNodeFound = false;

};


template<class graph_type>
bool Dijkstra<graph_type>::Search()
{
	//indexed priority queue to store the nodes of the edges of the frontier in order to automatically sort them by its total cost from the source nodeIndex
	//this type of queue takes the index of the nodeIndex to add it into the queue and it automatically checks its cost from the totalCostTo vector to sort them in order from the lowest to highest cost
	IndexedPriorityQLow<double> iPQ(totalCostTo, graph.GetNumNodes()); //the second parameter indicates the max number of elements

	//add the source nodeIndex into the queue to start the while-loop
	iPQ.insert(sourceNode);

	while (!iPQ.empty())
	{
		//index of the nodeIndex with the lowest cost from the edges in the frontier
		int nextNodeIndex = iPQ.Pop();

		//Add the edge that is currently pointing to the nodeIndex with the lowest cost. Each nodeIndex will have only one edge stored inside the SPT vector
		//For example, the edge pointing to nodeIndex 8 has the lowest total cost among the other edges of the frontier. Therefore, the edge pointing to 8 is stored inside the SPT and the nodeIndex 8 will have only this edge inside the SPT.
		shortestPathTree[nextNodeIndex] = frontier[nextNodeIndex];

		//check for termination
		if (nextNodeIndex == targetNode)
		{
			return true;
		}

		//iterate through all the adjacency edges of the nodeIndex
		graph_type::EdgeIterator edgeItr(graph, nextNodeIndex);
		for (const Edge* iEdge = edgeItr.Begin(); !edgeItr.End(); iEdge = edgeItr.NextEdge())
		{
			//if pointed nodeIndex not yet in the SPT (the cost of the pointed nodeIndex is still zero)
			if(totalCostTo[iEdge->ToNode()] == 0)
			{
				//store the accoumulated cost from source to the pointed nodeIndex
				totalCostTo[iEdge->ToNode()] = totalCostTo[iEdge->FromNode()] + iEdge->GetCost();

				//add the pointed nodeIndex into the queue to automatically sort the stored nodeIndex from the lowest cost to the highest
				iPQ.insert(iEdge->ToNode());

				//Add the edge into the frontier vector
				frontier[iEdge->ToNode()] = iEdge;
			}
			///Edge Relaxation -> if best path targetNodeFound, 
			//if the search finds a better path to the nodeIndex
			else if (totalCostTo[iEdge->ToNode()] > totalCostTo[iEdge->FromNode()] + iEdge->GetCost() && shortestPathTree[iEdge->ToNode()] == NULL)
			{
				totalCostTo[iEdge->ToNode()] = totalCostTo[iEdge->FromNode()] + iEdge->GetCost();

				//Update the priority queue with the new cost
				iPQ.ChangePriority(iEdge->ToNode());

				frontier[iEdge->ToNode()] = iEdge;
			}
		}
	}

	return false;

}
