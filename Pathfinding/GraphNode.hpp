#pragma once

/**
*	GraphNode.hpp
*
*	Class to store the minimum information of the graph node
*
*/


///<summary>GraphNode class stores the minimum information of the nodeIndex that it requires for the adjacency list graph representation</summary>
//In this case, an Index.
class GraphNode
{

public:

	///<summary>Node index sets to -1, a non valid index</summary>
	GraphNode() : nodeIndex(-1) {}
	GraphNode(int index) : nodeIndex(index) {}
	virtual ~GraphNode() {}

	int GetIndex() const { return nodeIndex; }
	void SetIndex(int index) { nodeIndex = index; }

protected:
	//The index of the nodeIndex. valid index are >= 0
	int nodeIndex;

};
