#pragma once

#include <fstream>


///<summary>class used to store the basic info of the edges: the nodes that connect and the cost of traveling</summary>
class GraphEdge
{

public:

	///<summary>Information not passed</summary>
	GraphEdge() : edgeCost(1.0), fromNode(-1), toNode(-1) {}
	GraphEdge(int from, int to) : edgeCost(0.0), fromNode(from), toNode(to) {}
	GraphEdge(int from, int to, double cost) : edgeCost(cost), fromNode(from), toNode(to) {}
	GraphEdge(std::ifstream& stream)
	{
		char buffer[50];
		stream >> buffer >> fromNode >> buffer >> toNode >> buffer >> edgeCost;

	}

	~GraphEdge() {}

	int FromNode() const { return fromNode; }
	void SetFromNode(int from) { fromNode = from; }

	int ToNode() const { return toNode; }
	void SetToNode(int to) { toNode = to; }

	double GetCost() const { return edgeCost; }
	void SetEdgeCost(int cost) { edgeCost = cost; }

private:

	//nodes indices
	int fromNode;
	int toNode;

	//cost of traveling
	double edgeCost;

};