#include "RTSCommon/AISystem/Utilities/PathPlanner.hpp"
#include "RTSCommon/AISystem/Utilities/CellSpacePartition.hpp"
#include "RTSCommon/AISystem/GraphSystem/Graph_AStar.hpp"
#include "RTSCommon/AISystem/Utilities/Regulator.hpp"
#include "RTSCommon/Components/TransformComponent.hpp"
#include "RTSCommon/Entity/Entity.hpp"
#include <iostream>


PathPlanner::PathPlanner(Common::Entity *owner, CellSpacePartition<Common::Entity>& mapCells)
	: owner(owner), mapCells(mapCells)
{}


PathPlanner::PathPlanner(CellSpacePartition<Common::Entity>& map) : mapCells(map)
{}


void PathPlanner::ConvertIndicesToVectors(std::list<int> indices, std::list<glm::vec2>& path, int layer)
{
	for (int i : indices)
	{
		path.push_front(mapCells.GetLayer(layer).GetNode(i).GetNodePosition());
	}

}


void PathPlanner::PathSmoothing(std::list<glm::vec2>& path)
{
	std::list<glm::vec2>::iterator node1, node2;
	node1 = path.begin();

	while (*node1 != path.back())
	{
		node2 = node1;
		node2++;
		if (node2 == path.end())
		{
			return;
		}
		node2++;

		while (node2 != path.end())
		{
			if (mapCells.AreOnSight(*node1, *node2))
			{
				node2 = path.erase(++node1, node2);
				node1 = node2++;
				node1--;
			}
			else
			{
				++node2;
			}
		}
		++node1;
	}

}


bool PathPlanner::SquadHierarchicalSearch(glm::vec2 sourcePos, glm::vec2 target, std::list<glm::vec2>& path)
{
	if (sourcePos == glm::vec2())
	{
		return false;
	}

	int layer = mapCells.GetNumLayers();

	glm::vec2 targetPos = target;

	//Layers must have clusters big as half of the higher layer
	while (layer >= 0)
	{
		if (layer == 0 && mapCells.AreOnSight(sourcePos, targetPos))
		{
			path.clear();
			path.push_front(targetPos);
			return true;
		}

		path.clear();

		if (layer > 0 && mapCells.GetCluster(sourcePos, layer) == mapCells.GetCluster(targetPos, layer))
		{
			--layer;
			continue;
		}

		int numNodes = mapCells.GetLayer(layer).GetNumNodes();

		int sourceNode = mapCells.InsertNodeAtLayer(layer, sourcePos);
		int targetNode = mapCells.InsertNodeAtLayer(layer, targetPos);

		AStar<Graph<NavGraphNode<int>, GraphEdge>> AStar(mapCells.GetLayer(layer), mapCells.GetCellSize(), sourceNode, targetNode);
		ConvertIndicesToVectors(AStar.GetPathToTarget(), path, layer);

		if (layer == 0)
		{
			PathSmoothing(path);
			path.pop_front();
			return true;
		}

		if (path.size() > 2)
		{
			//Smooth the path by removing nodes within the same clusters
			glm::vec2& clusterTopLeft = mapCells.GetCluster(sourcePos, layer)->topLeft;
			glm::vec2& clusterBottomRight = mapCells.GetCluster(sourcePos, layer)->bottomRight;
			glm::vec2& nodePos = *std::next(path.begin(), 2);
			if (nodePos.x > clusterTopLeft.x && nodePos.x < clusterBottomRight.x &&
				nodePos.y > clusterTopLeft.y && nodePos.y < clusterBottomRight.y)
			{
				path.erase(std::next(path.begin()));
			}
		}
		if (path.size() > 2)
		{
			glm::vec2& clusterTopLeft = mapCells.GetCluster(targetPos, layer)->topLeft;
			glm::vec2& clusterBottomRight = mapCells.GetCluster(targetPos, layer)->bottomRight;
			glm::vec2& nodePos = *std::next(path.begin(), path.size() - 3);
			if (nodePos.x > clusterTopLeft.x && nodePos.x < clusterBottomRight.x &&
				nodePos.y > clusterTopLeft.y && nodePos.y < clusterBottomRight.y)
			{
				path.erase(std::next(path.begin(), path.size() - 2));
			}
		}

		if (path.size() > 1)
		{
			if (mapCells.GetClosestNodeToPosition(path.front()) == mapCells.GetClosestNodeToPosition(*std::next(path.begin())))
			{
				path.pop_front();
			}

			targetPos = *std::next(path.begin());
		}
		else
		{
			targetPos = path.front();
		}

		//remove S and G from abstract graph
		if (numNodes < mapCells.GetLayer(layer).GetNumNodes())
		{
			if (numNodes == mapCells.GetLayer(layer).GetNumNodes() - 2)
			{
				mapCells.GetLayer(layer).RemoveLastNode();
			}

			if (numNodes == mapCells.GetLayer(layer).GetNumNodes() - 1)
			{
				mapCells.GetLayer(layer).RemoveLastNode();
			}
		}

		--layer;
	}

	return true;

}


bool PathPlanner::UnitHierarchicalSearch(glm::vec2 target, std::list<glm::vec2>& path)
{
	if (owner->GetComponent<Common::TransformComponent>() == nullptr)
	{
		return false;
	}

	int layer = mapCells.GetNumLayers();

	glm::vec2 sourcePos = owner->GetComponent<Common::TransformComponent>()->GetPosition();
	glm::vec2 targetPos = target;

	//Layers must have clusters big as half of the higher layer
	while (layer >= 0)
	{
		if (layer == 0 && mapCells.AreOnSight(owner->GetComponent<Common::TransformComponent>()->GetPosition(), targetPos))
		{
			path.clear();
			path.push_front(targetPos);
			return true;
		}

		path.clear();
	
		if (layer > 0 && mapCells.GetCluster(sourcePos, layer) == mapCells.GetCluster(targetPos, layer))
		{
			--layer;
			continue;
		}

		int numNodes = mapCells.GetLayer(layer).GetNumNodes();

		int sourceNode = mapCells.InsertNodeAtLayer(layer, sourcePos);
		int targetNode = mapCells.InsertNodeAtLayer(layer, targetPos);

		AStar<Graph<NavGraphNode<int>, GraphEdge>> AStar(mapCells.GetLayer(layer), mapCells.GetCellSize(), sourceNode, targetNode);
		ConvertIndicesToVectors(AStar.GetPathToTarget(), path, layer);

		if (layer == 0)
		{
			PathSmoothing(path);
			path.pop_front();
			return true;
		}

		if (path.size() > 2)
		{
			//Smooth the path by removing nodes within the same clusters
			glm::vec2& clusterTopLeft = mapCells.GetCluster(sourcePos, layer)->topLeft;
			glm::vec2& clusterBottomRight = mapCells.GetCluster(sourcePos, layer)->bottomRight;
			glm::vec2& nodePos = *std::next(path.begin(), 2);
			if (nodePos.x > clusterTopLeft.x && nodePos.x < clusterBottomRight.x &&
				nodePos.y > clusterTopLeft.y && nodePos.y < clusterBottomRight.y)
			{
				path.erase(std::next(path.begin()));
			}
		}
		if (path.size() > 2)
		{
			glm::vec2& clusterTopLeft = mapCells.GetCluster(targetPos, layer)->topLeft;
			glm::vec2& clusterBottomRight = mapCells.GetCluster(targetPos, layer)->bottomRight;
			glm::vec2& nodePos = *std::next(path.begin(), path.size() - 3);
			if (nodePos.x > clusterTopLeft.x && nodePos.x < clusterBottomRight.x &&
				nodePos.y > clusterTopLeft.y && nodePos.y < clusterBottomRight.y)
			{
				path.erase(std::next(path.begin(), path.size() - 2));
			}
		}

		if (path.size() > 1)
		{
			if (mapCells.GetClosestNodeToPosition(path.front()) == mapCells.GetClosestNodeToPosition(*std::next(path.begin())))
			{
				path.pop_front();
			}

			targetPos = *std::next(path.begin());
		}
		else
		{
			targetPos = path.front();
		}

		//remove S and G from abstract graph
		if (numNodes < mapCells.GetLayer(layer).GetNumNodes())
		{
			if (numNodes == mapCells.GetLayer(layer).GetNumNodes() - 2)
			{
				mapCells.GetLayer(layer).RemoveLastNode();
			}

			if (numNodes == mapCells.GetLayer(layer).GetNumNodes() - 1)
			{
				mapCells.GetLayer(layer).RemoveLastNode();
			}
		}

		--layer;
	}

	return true;

}


bool PathPlanner::FindPathToPosition(glm::vec2 target, std::list<glm::vec2>& path)
{
	if (owner->GetComponent<Common::TransformComponent>() == nullptr)
	{
		return false;
	}


	if (mapCells.AreOnSight(owner->GetComponent<Common::TransformComponent>()->GetPosition(), target))
	{
		path.push_back(target);
		return true;
	}

	int sourceNode = mapCells.GetClosestNodeToPosition(owner->GetComponent<Common::TransformComponent>()->GetPosition());

	if (sourceNode == node_not_found)
	{
		return false;
	}

	int targetNode = mapCells.GetClosestNodeToPosition(target);

	if (targetNode == node_not_found)
	{
		return false;
	}

	AStar<Graph<NavGraphNode<int>, GraphEdge>> AStar(mapCells.GetNavGraph(), mapCells.GetCellSize(), sourceNode, targetNode);

	ConvertIndicesToVectors(AStar.GetPathToTarget(), path);

	if (!path.empty())
	{
		path.push_back(target);
		return true;
	}
	else
	{
		return false;
	}

	return false;

}


bool PathPlanner::FindLeastCostPathToItemType(unsigned int itemType)
{
	//TODO use dijkstra to find the item of the given type with the best path
	return false;

}
