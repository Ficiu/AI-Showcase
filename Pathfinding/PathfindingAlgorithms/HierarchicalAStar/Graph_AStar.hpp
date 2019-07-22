#pragma once

/**
*	Graph_AStar.hpp
*
*	Template class to execute A* algorithm
*
*/

#include "RTSCommon/AISystem/Pathfinding/Graph.hpp"
#include "RTSCommon/AISystem/PriorityQueue.hpp"

#include <glm/glm.hpp>
#include <vector>
#include <list>


template <class graph_type>
class AStar
{

private:

	enum { VISITED, UNVISITED, NO_PARENT };
	typedef typename graph_type::EdgeType Edge;
	typedef typename graph_type::NodeType Node;

public:

	AStar(const graph_type& graph, glm::vec2 cellSize, int source, int target = -1) :
		graph(graph), realCostTo(graph.GetNumNodes()), FCost(graph.GetNumNodes()), frontier(graph.GetNumNodes()),
		shortestPathTree(graph.GetNumNodes()), sourceNode(source), targetNode(target)
	{
		tieBreaker = 1.01f;
		diagonalFactor = glm::length(cellSize) - (cellSize.x + cellSize.y);
		FindPath();

	}

	AStar(const graph_type& graph, glm::vec2 cellSize, glm::vec2 topleft, glm::vec2 bottomright, int source, int target = -1) :
		graph(graph), realCostTo(graph.GetNumNodes()), FCost(graph.GetNumNodes()), frontier(graph.GetNumNodes()),
		shortestPathTree(graph.GetNumNodes()), clusterTopLeft(topleft), clusterBottomRight(bottomright),
		sourceNode(source), targetNode(target)
	{
		tieBreaker = 1.01f;
		diagonalFactor = glm::length(cellSize) - (cellSize.x + cellSize.y);
		FindIntraClusterPath();

	}

	~AStar() {}

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

	double GetTotalCostToTarget() const { return realCostTo[targetNode]; }
	bool HasFoundTarget() const { return targetNodeFound; }

private:

	void FindPath();
	void FindIntraClusterPath();
	IndexedPriorityQLow<double> InitializeIndexedPQ();
	void ExpandSearch(IndexedPriorityQLow<double>&);
	void ExpandClusterSearch(IndexedPriorityQLow<double>& iPQ);
	void InsertNeighbourNodesOnTheFrontier(const Edge *edge, IndexedPriorityQLow<double> &pq);
	void CalculateGCostAndFCost(const Edge* edge);
	double CalculateHeuristic(const int &toNode);
	bool IsNodeInsideCluster(int toNode);

private:

	const graph_type& graph;

	///<summary>holds the total cost of the best path from any nodeIndex to the source. The vector is indexed into by nodeIndex index.
	///example: realCostTo[5] holds the total cost the edges of the best path targetNodeFound so far between nodeIndex 5 and the source nodeIndex</summary>
	std::vector<double> realCostTo;

	///<summary>holds the estimated cost of the path from each nodeIndex to the target. The vector is indexed into by nodeIndex index.</summary>
	///This vector will be used to direct the frontier towards the target, instead of equally search in every direction.
	std::vector<double> FCost;

	///<summary>the adjacency edges leading to nodes not yet into the SPT. vector indexed into by nodeIndex index
	///it stores "parent" edges leading to nodes connected to the SPT but not inside of it yet</summary>
	std::vector<const Edge*> frontier;
	///<summary>the edges for the shortest path tree. it encapsulates the best path from any nodeIndex in the SPT to the source nodeIndex.
	///stores the edges of nodes of the SPT</summary>
	std::vector<const Edge*> shortestPathTree;

	glm::vec2 clusterTopLeft;
	glm::vec2 clusterBottomRight;

	int sourceNode, targetNode;

	double diagonalFactor;
	float tieBreaker;

	bool targetNodeFound = false;

};


template<class graph_type>
void AStar<graph_type>::FindPath()
{
	//OPEN list
	IndexedPriorityQLow<double> iPQ = InitializeIndexedPQ();

	while (!iPQ.empty() && !targetNodeFound)
	{
		ExpandSearch(iPQ);
	}

}


template<class graph_type>
IndexedPriorityQLow<double> AStar<graph_type>::InitializeIndexedPQ()
{
	//indexed priority queue to store the nodes of the edges of the frontier in order to automatically sort them by its estimated total cost from the source nodeIndex to the target nodeIndex
	//The priority queue will return the nodeIndex with the lowest estimated total cost, which should be the closest nodeIndex to the target among the frontier ones.
	IndexedPriorityQLow<double> pq(FCost, graph.GetNumNodes());

	//add the source nodeIndex into the queue to start the while-loop
	pq.insert(sourceNode);

	return pq;

}


template<class graph_type>
void AStar<graph_type>::ExpandSearch(IndexedPriorityQLow<double>& iPQ)
{
	//index of the nodeIndex with the lowest cost from the edges in the frontier. The closest nodeIndex to the target
	int lowestFCostNode = iPQ.Pop();

	//Add the edge that is currently pointing to the nodeIndex with the lowest cost. Each nodeIndex will have only one edge stored inside the SPT vector
	//For example, the edge pointing to nodeIndex 8 has the lowest total cost among the other edges of the frontier. Therefore, the edge pointing to 8 is stored inside the SPT and the nodeIndex 8 will have only this edge inside the SPT.
	//Put the examined node into the CLOSED list
	shortestPathTree[lowestFCostNode] = frontier[lowestFCostNode];

	//check for termination
	if (lowestFCostNode == targetNode)
	{
		targetNodeFound = true;
		return;
	}

	//iterate through all the adjacency edges of the nodeIndex
	graph_type::EdgeIterator edgeItr(graph, lowestFCostNode);
	for (const Edge* iEdge = edgeItr.Begin(); !edgeItr.End(); iEdge = edgeItr.NextEdge())
	{
		InsertNeighbourNodesOnTheFrontier(iEdge, iPQ);
	}

}


template<class graph_type>
void AStar<graph_type>::FindIntraClusterPath()
{
	//OPEN list
	IndexedPriorityQLow<double> iPQ = InitializeIndexedPQ();

	while (!iPQ.empty() && !targetNodeFound)
	{
		ExpandClusterSearch(iPQ);
	}

}


template<class graph_type>
void AStar<graph_type>::ExpandClusterSearch(IndexedPriorityQLow<double>& iPQ)
{
	//index of the nodeIndex with the lowest cost from the edges in the frontier. The closest nodeIndex to the target
	int lowestFCostNode = iPQ.Pop();

	//Add the edge that is currently pointing to the nodeIndex with the lowest cost. Each nodeIndex will have only one edge stored inside the SPT vector
	//For example, the edge pointing to nodeIndex 8 has the lowest total cost among the other edges of the frontier. Therefore, the edge pointing to 8 is stored inside the SPT and the nodeIndex 8 will have only this edge inside the SPT.
	//Put the examined node into the CLOSED list
	shortestPathTree[lowestFCostNode] = frontier[lowestFCostNode];

	//check for termination
	if (lowestFCostNode == targetNode)
	{
		targetNodeFound = true;
		return;
	}

	//iterate through all the adjacency edges of the nodeIndex
	graph_type::EdgeIterator edgeItr(graph, lowestFCostNode);
	for (const Edge* iEdge = edgeItr.Begin(); !edgeItr.End(); iEdge = edgeItr.NextEdge())
	{
		if (IsNodeInsideCluster(iEdge->ToNode()))
		{
			InsertNeighbourNodesOnTheFrontier(iEdge, iPQ);
		}
	}

}


template<class graph_type>
bool AStar<graph_type>::IsNodeInsideCluster(int toNode)
{
	glm::vec2 nodePos = graph.GetNode(toNode).GetNodePosition();
	if (nodePos.x >= clusterTopLeft.x && nodePos.x < clusterBottomRight.x && nodePos.y >= clusterTopLeft.y && nodePos.y < clusterBottomRight.y)
	{
		return true;
	}

	return false;

}


template<class graph_type>
void AStar<graph_type>::InsertNeighbourNodesOnTheFrontier(const Edge *iEdge, IndexedPriorityQLow<double>& iPQ)
{
	//if pointed nodeIndex not yet in the SPT/CLOSED list (the Gcost of the pointed node is zero)
	if (realCostTo[iEdge->ToNode()] == 0)
	{
		CalculateGCostAndFCost(iEdge);

		///OPEN list
		//add the pointed nodeIndex into the queue to automatically sort the stored nodeIndex from the lowest cost to the highest
		iPQ.insert(iEdge->ToNode());

		//Add the edge into the frontier vector
		frontier[iEdge->ToNode()] = iEdge;
	}
	///Edge Relaxation -> if best path targetNodeFound, 
	//if the search finds a better path to the nodeIndex
	else if (realCostTo[iEdge->ToNode()] > realCostTo[iEdge->FromNode()] + iEdge->GetCost() && shortestPathTree[iEdge->ToNode()] == NULL)
	{
		CalculateGCostAndFCost(iEdge);

		///OPEN list
		//Update the priority queue with the new cost
		iPQ.ChangePriority(iEdge->ToNode());

		//Add the edge into the frontier vector
		frontier[iEdge->ToNode()] = iEdge;
	}

}


template<class graph_type>
double AStar<graph_type>::CalculateHeuristic(const int &node)
{
	float dx = std::fabs((graph.GetNode(targetNode).GetNodePosition().x -
		graph.GetNode(node).GetNodePosition().x));

	float dy = std::fabs((graph.GetNode(targetNode).GetNodePosition().y -
		graph.GetNode(node).GetNodePosition().y));

	//Octile distance and tie breaking
	return ((dx + dy) + diagonalFactor * (std::min)(dx * 0.1, dy * 0.1)) *
		tieBreaker;

}


template<class graph_type>
void AStar<graph_type>::CalculateGCostAndFCost(const Edge* edge)
{
	//store the accoumulated cost from source to the pointed nodeIndex
	realCostTo[edge->ToNode()] = realCostTo[edge->FromNode()] + edge->GetCost();
	//the estimated total cost of the path from the source nodeIndex, passing by the current pointed nodeIndex, to the target
	FCost[edge->ToNode()] = realCostTo[edge->ToNode()] + CalculateHeuristic(edge->ToNode());

}
