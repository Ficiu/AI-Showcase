#pragma once

/**
*	NavGraphNode.hpp
*
*	Class to store node position and extra info
*
*/


#include "RTSCommon\AISystem\Pathfinding\GraphNode.hpp"

#include <fstream>
#include <glm/glm.hpp>


///<summary>class to store additional information of the nodeIndex, such as its position</summary>
template<class extra_info = void*>
class NavGraphNode : public GraphNode
{

public:

	NavGraphNode() : extraInfo(extra_info()) {}
	NavGraphNode(int i, glm::vec2 position) : GraphNode(i), nodePosition(position), extraInfo(extra_info()) {}
	NavGraphNode(std::ifstream& stream)
	{
		char buffer[50];
		stream >> buffer >> nodeIndex >> buffer >> nodePosition.x >> buffer >> nodePosition.y;

	}

	virtual ~NavGraphNode() {}

	glm::vec2 GetNodePosition() const { return nodePosition; }
	void SetNodePosition(glm::vec2 position) { nodePosition = position; }
	extra_info GetNodeInfo() const { return extraInfo; }
	void SetNodeInfo(extra_info info) { extraInfo = info; }

private:

	//Node position
	glm::vec2 nodePosition;
	
	///<summary>variable in case the nodeIndex needs extra information, such as a pointer to an object situated on the nodeIndex</summary>
	extra_info extraInfo;

};