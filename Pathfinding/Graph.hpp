#pragma once

/**
*	Graph.hpp
*
*	Template class to create and store the graph
*
*/

#include <cassert>
#include <vector>
#include <list>
#include <fstream>
#include <glm/glm.hpp>


template<class node_type, class edge_type>
class Graph
{

public:

	typedef node_type			NodeType;
	typedef edge_type			EdgeType;
	typedef std::list<EdgeType>	EdgeList;

	///<summary>digraph</summary>
	Graph() : digraph(false), nextNodeIndex(0) {}
	Graph(bool digraph) : digraph(digraph), nextNodeIndex(0) {}

	//Return the nodeIndex
	const NodeType& GetNode(int i) const;
	NodeType& GetNode(int i);

	//return edge
	const EdgeType& GetEdge(int from, int to) const;
	EdgeType& GetEdge(int from, int to);

	//return index of the next free nodeIndex
	int GetNextFreeNodeIndex() const { return nextNodeIndex; }

	//add a nodeIndex and return its index. It adds a nodeIndex to the end of the nodeIndex vector and an edge list to the end of edge list vector
	int InsertNode(NodeType node);
	//remove a nodeIndex (set its index to invalid/-1) and any links to neighbouring nodes
	void RemoveNode(int node);

	void RemoveLastNode();

	//Add the edge to the adjacency edge list. it can add a second one if not diagraph (normal graph)
	void InsertEdge(EdgeType edge);
	//remove the edge from the adjacency edge list. it can remove a second one if not diagraph (normal graph)
	void RemoveEdge(int from, int to);
	//remove all the edges
	void RemoveEdges();

	//total number of active and inactive nodes
	int GetNumNodes() const { return (int)nodeVector.size(); }
	int GetNumActiveNodes() const;
	int GetNumEdges() const;

	bool IsDigraph() const { return digraph; }
	bool IsEmpty() const { return nodeVector.empty(); }

	//check if the nodeIndex/edge is in the graph
	bool IsNodePresent(int idx) const;
	int IsNodePresent(glm::vec2 position) const;
	bool IsEdgePresent(int from, int to) const;

	//Clear the graph
	void Clear();

	void Load(std::ifstream& stream);
	void Save(const char* filename);

	///class used to iterate through all the edges of a specific nodeIndex
	class EdgeIterator
	{

	public:

		EdgeIterator(const Graph<NodeType, edge_type> &graph, int node) : graph(graph), index(node)
		{
			edgeItr = graph.edgeList[index].begin();

		};

		~EdgeIterator() {};

		const EdgeType* Begin()
		{
			edgeItr = graph.edgeList[index].begin();
			if (End())
			{
				return NULL;
			}

			return &(*edgeItr);

		}

		const EdgeType* NextEdge()
		{
			++edgeItr;
			if (End())
			{
				return NULL;
			}

			return &(*edgeItr);

		}

		bool End()
		{
			return (edgeItr == graph.edgeList[index].end());

		}

	private:

		//reference to the graph and index of the nodeIndex to examine
		const Graph<NodeType, edge_type> &graph;
		const int index;

		//Edge iterator to iterate through all the edges of the indexed nodeIndex
		typename EdgeList::const_iterator edgeItr;

	};

	friend class EdgeIterator;

private:

	//Avoid to store same edge multiple times
	bool IsEdgeUnique(int from, int to) const;

	//remove all the edges that point to an invalid nodeIndex
	void RemoveInvalidEdges();

	///vector of nodes
	std::vector<NodeType> nodeVector;
	///<summary>vector of adjacency edge lists</summary>
	std::vector<EdgeList> edgeList;

	//is this a digraph? digraph = graph with directed edges
	bool digraph;

	//index of the next nodeIndex to be added
	int nextNodeIndex;

};


//Avoid to store same edge multiple times
template<class node_type, class edge_type>
bool Graph<node_type, edge_type>::IsEdgeUnique(int from, int to) const
{
	if (IsNodePresent(from) && IsNodePresent(to))
	{
		for (EdgeList::const_iterator edge = edgeList[from].begin(), endEdge = edgeList[from].end(); edge != endEdge; ++edge)
		{
			if (edge->ToNode() == to)
			{
				return false;
			}
		}
		return true;
	}

	return true;

}


//remove all the edges of an invalid nodeIndex
template<class node_type, class edge_type>
void Graph<node_type, edge_type>::RemoveInvalidEdges()
{
	for (size_t i = 0; i != edgeList.size(); ++i)
	{
		for (EdgeList::iterator edge = edgeList[i].begin(); edge != edgeList[i].end(); )
		{
			if (nodeVector[edge->FromNode()].GetIndex() == -1 || nodeVector[edge->ToNode()].GetIndex() == -1)
			{
				edge = edgeList[i].erase(edge); //TO DO: check if the list needs to be a const_iterator as well
			}
			else
			{
				++edge;
			}
		}
	}

}


template<class node_type, class edge_type>
const node_type& Graph<node_type, edge_type>::GetNode(int i) const
{
	assert(i < nodeVector.size() && i >= 0 && "Graph::GetNode: invalid index");

	return nodeVector[i];
	
}


template<class node_type, class edge_type>
node_type& Graph<node_type, edge_type>::GetNode(int i)
{
	assert(i < nodeVector.size() && i >= 0 && "Graph::GetNode: invalid index");

	return nodeVector[i];

}


template<class node_type, class edge_type>
const edge_type& Graph<node_type, edge_type>::GetEdge(int from, int to) const
{
	assert(from < nodeVector.size() && from >= 0 && "GetEdge: invalid 'from' index");
	assert(to < nodeVector.size() && to >= 0 && "GetEdge: invalid 'to' index");

	for (EdgeList::const_iterator edge = edgeList[from].begin(), end = edgeList[from].end(); edge != end; ++edge)
	{
		if (edge->ToNode() == to)
		{
			return *edge;
		}
	}

	assert(0 && "GetEdge: edge not present");

}


template<class node_type, class edge_type>
edge_type& Graph<node_type, edge_type>::GetEdge(int from, int to)
{
	assert(from < nodeVector.size() && from >= 0 && "GetEdge: invalid 'from' index");
	assert(to < nodeVector.size() && to >= 0 && "GetEdge: invalid 'to' index");

	for (EdgeList::iterator edge = edgeList[from].begin(), end = edgeList[from].end(); edge != end; ++edge)
	{
		if (edge->ToNode() == to)
		{
			return *edge;
		}
	}

	assert(0 && "GetEdge: edge not present");
	return *(new edge_type());

}


//add a nodeIndex and return its index. It adds a nodeIndex to the end of the nodeIndex vector and an edge list to the end of edge list vector
template<class node_type, class edge_type>
int Graph<node_type, edge_type>::InsertNode(NodeType node)
{
	//if a nodeIndex with the same index is already stored
	if (size_t(node.GetIndex()) < nodeVector.size())
	{
		//if the stored nodeIndex has an invalid index, move on. Otherwise call assert
		assert(nodeVector[node.GetIndex()].GetIndex() == -1 && "InsertNode: Node with the same index already stored");

		//change nodeIndex
		nodeVector[node.GetIndex()] = node;

		return nextNodeIndex;
	}
	else
	{
		assert(node.GetIndex() == nextNodeIndex && "InsertNode: Node not indexed correctly");

		nodeVector.push_back(node);
		edgeList.push_back(EdgeList());

		return nextNodeIndex++;
	}

}


//remove a nodeIndex (set its index to invalid/-1) and any links to neighbouring nodes
template<class node_type, class edge_type>
void Graph<node_type, edge_type>::RemoveNode(int node)
{
	assert(node >= 0 && node < nodeVector.size() && "RemoveNode: invalid index");

	nodeVector[node].SetIndex(-1);

	//slow process
	if (digraph)
	{
		RemoveInvalidEdges();
	}
	else //fast process
	{		
		//remove all the edges to the nodeIndex
		for (size_t i = 0; i != edgeList.size(); ++i)
		{
			for (EdgeList::const_iterator edge = edgeList[i].begin(); edge != edgeList[i].end(); ++edge)
			{
				if (edge->ToNode() == node)
				{
					edge = edgeList[i].erase(edge); //TO DO: check if the list needs to be a const_iterator as well
					break;
				}
			}
		}

		//remove all the edges from the ndoe
		edgeList[node].clear();
	}

}


template<class node_type, class edge_type>
void Graph<node_type, edge_type>::RemoveLastNode()
{
	RemoveNode((int)nodeVector.size() - 1);

	nodeVector.pop_back();
	edgeList.pop_back();
	--nextNodeIndex;
}


//Add the edge to the adjacency edge list. it can add a second one if not diagraph (normal graph)
template<class node_type, class edge_type>
void Graph<node_type, edge_type>::InsertEdge(EdgeType edge)
{
	//check if the edge is pointing to nodes existing in the graph
	assert(edge.FromNode() < nextNodeIndex && edge.ToNode() < nextNodeIndex && "InsertEdge: Edge with invalid indices");

	//check if nodes have valid index
	if (nodeVector[edge.FromNode()].GetIndex() != -1 && nodeVector[edge.ToNode()].GetIndex() != -1)
	{
		//check if the same edge was already stored in the vector of adjacency edge lists
		if (IsEdgeUnique(edge.FromNode(), edge.ToNode()))
		{
			//push back the edhe into the list of edges departing from the right nodeIndex
			edgeList[edge.FromNode()].push_back(edge);
		}

		//if graph is undirected, add the edge pointing in the opposite direction
		if (!digraph)
		{
			//check if edge is unique
			if (IsEdgeUnique(edge.ToNode(), edge.FromNode()))
			{
				EdgeType newEdge = edge;
				newEdge.SetFromNode(edge.ToNode());
				newEdge.SetToNode(edge.FromNode());
				edgeList[edge.ToNode()].push_back(newEdge);
			}
		}
	}

}


//remove the edge from the adjacency edge list. it can remove a second one if not diagraph (normal graph)
template<class node_type, class edge_type>
void Graph<node_type, edge_type>::RemoveEdge(int from, int to)
{
	assert(from < nodeVector.size() && to < nodeVector.size() && "RemoveEdge: invalid nodeIndex indices");

	for (EdgeList::const_iterator edge = edgeList[from].begin(); edge != edgeList[from].end(); ++edge)
	{
		if (edge->ToNode() == to)
		{
			edge = edgeList[from].erase(edge);
			break;
		}
	}

	if (!digraph)
	{
		for (EdgeList::const_iterator edge = edgeList[to].begin(); edge != edgeList[to].end(); ++edge)
		{
			if (edge->ToNode() == from)
			{
				edge = edgeList[to].erase(edge);
				break;
			}
		}
	}

}


template<class node_type, class edge_type>
void Graph<node_type, edge_type>::RemoveEdges()
{
	for (size_t i = 0, size = edgeList.size(); i != size; ++i)
	{
		edgeList[i].clear();
	}

}


template<class node_type, class edge_type>
int Graph<node_type, edge_type>::GetNumActiveNodes() const
{
	unsigned int count = 0;
	for (size_t i = 0, size = nodeVector.size(); i != size; ++i)
	{
		if (nodeVector[i].GetIndex() != -1)
		{
			++count;
		}
	}

	return count;

}


template<class node_type, class edge_type>
int Graph<node_type, edge_type>::GetNumEdges() const
{
	unsigned int count = 0;

	for (size_t i = 0, size = edgeList.size(); i != size; ++i)
	{
		for (EdgeList::const_iterator edge = edgeList[i].begin(), end = edgeList[i].end(); edge != end; ++edge)
		{
			++count;
		}
	}

	return count;

}


template<class node_type, class edge_type>
bool Graph<node_type, edge_type>::IsNodePresent(int idx) const
{
	if (size_t(idx) >= nodeVector.size() || nodeVector[idx].GetIndex() == -1)
	{
		return false;
	}
	else return true;

}


template<class node_type, class edge_type>
int Graph<node_type, edge_type>::IsNodePresent(glm::vec2 position) const
{
	for (int i = 0; i != nextNodeIndex; ++i)
	{
		if (position == nodeVector[i].GetNodePosition())
		{
			return nodeVector[i].GetIndex();
		}
	}

	return -1;

}


template<class node_type, class edge_type>
bool Graph<node_type, edge_type>::IsEdgePresent(int from, int to) const
{
	if (IsNodePresent(from) && IsNodePresent(to))
	{
		for (EdgeList::const_iterator edge = edgeList[from].begin(), end = edgeList[from].end(); edge != end; ++edge)
		{
			if (edge->ToNode() == to)
			{
				return true;
			}
		}
		return false;
	}

	return false;

}


//Clear the graph
template<class node_type, class edge_type>
void Graph<node_type, edge_type>::Clear()
{
	nextNodeIndex = 0;
	nodeVector.clear();
	edgeList.clear();

}


template<class node_type, class edge_type>
void Graph<node_type, edge_type>::Load(std::ifstream& stream)
{
	//Prepare the graph
	Clear();

	unsigned int numNodes, numEdges;

	//Get n° of nodes
	stream >> numNodes;

	for (int i = 0; i < numNodes; ++i)
	{
		//Create nodeIndex from stream
		NodeType node(stream);

		//if invalid index
		if (node.GetIndex() == -1)
		{
			//push the invalid nodeIndex and an empty edge list into the vectors
			nodeVector.push_back(node);
			edgeList.push_back(EdgeList());

			++nextNodeIndex;
		}
		else
		{
			//add a valid nodeIndex at the end of the nodeIndex stack or change an invalid nodeIndex with same index
			InsertNode(node);
		}
	}

	//n° of edges
	stream >> numEdges;

	for (int i = 0; i < numEdges; ++i)
	{
		//create edge from stream
		EdgeType edge(stream);

		//add the edge to the right adjacency edge list and its opposite edge in case digraph
		InsertEdge(edge);
	}

}


template<class node_type, class edge_type>
void Graph<node_type, edge_type>::Save(const char* filename)
{
	std::ofstream ostrm(filename);
	
	ostrm << nodeVector.size() << std::endl;

	for (size_t i = 0, size = nodeVector.size(); i < size; ++i)
	{
		ostrm << "Index: " << nodeVector[i].GetIndex() << " posX: " << nodeVector[i].GetNodePosition().x << " posY: " << nodeVector[i].GetNodePosition().y << std::endl;
	}

	ostrm << GetNumEdges() << std::endl;

	for (size_t i = 0, size = edgeList.size(); i < size; ++i)
	{
		for (EdgeList::const_iterator edgeItr = edgeList[i].begin(), end = edgeList[i].end(); edgeItr != end; ++edgeItr)
		{
			ostrm << "From: " << edgeItr->FromNode() << " To: " << edgeItr->ToNode() << " Cost: " << edgeItr->GetCost() << std::endl;
		}
	}

	ostrm.close();

}
