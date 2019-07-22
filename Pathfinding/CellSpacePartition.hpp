#pragma once

/**
*	CellSpacePartition.hpp
*	
*	Template class to create the main navgraph and the subgraphs for the Hiearchical A*
*	and to give functionalities to the pathfinding algorithms (IE Line of sight)
*
*/

#include "RTSCommon/AISystem/Pathfinding/Graph.hpp"
#include "RTSCommon/AISystem/Pathfinding/GraphEdge.hpp"
#include "RTSCommon/AISystem/Pathfinding/PathfindingAlgorithms/HierarchicalAStar/Graph_AStar.hpp"
#include "RTSCommon/AISystem/Pathfinding/NavGraphNode.hpp"
#include "RTSCommon/Map/Map.hpp"

#include <glm/glm.hpp>


template <class node, class entity = void*>
struct NavGraphCell
{
	std::list<node*> nodeList;
	std::list<entity*> entityList;

	glm::vec2 cellPosition;
	glm::vec2 cellSize;
	int nodeIndex = -1;
	bool obstacle = false;

};


template <class entity = void*>
struct Cluster
{
	std::vector<std::vector<NavGraphCell<NavGraphNode<int>, entity>*>> clusterCells;
	std::vector<NavGraphNode<int>*> clusterNodes;
	std::vector<int> nodeToSearch;

	int layerID;
	int clusterID;
	glm::vec2 topLeft;
	glm::vec2 bottomRight;
	glm::vec2 clusterSize;

};


template <class entity = void*>
struct Entrance
{
	//the cells that form the entrance from two adjacent clusters
	std::vector<std::pair<NavGraphCell<NavGraphNode<int>, entity>*, NavGraphCell<NavGraphNode<int>, entity>*>> entranceCells;
	int lenght;
	int vertical;
	int horizontal;

	bool obstacleFound = false;

	int firstCluster;
	int secondCluster;

};


struct TemporaryClusterData
{
	//vectors to store temporary nodes that could be used by multiple agents
	std::vector<NavGraphNode<int>*> nodes;
	std::vector<std::vector<GraphEdge*>> temporaryEdges;

};


template <class entity_type>
class CellSpacePartition
{

private:

	typedef Graph<NavGraphNode<int>, GraphEdge> Graph;
	typedef std::vector<std::vector<Cluster<entity_type>>> clusterVector;
	//Hierarchical Pathfinding//
	enum { horizontal = 0, vertical };

public:

	CellSpacePartition(Common::Map* map, int maxNeighbors = 1) :
		neighbors(maxNeighbors, nullptr), nodeNeighbors(20, nullptr)
	{
		mapData = map;
		
		cellSizeX = mapData->GetMapData().tileSize.x;
		cellSizeY = mapData->GetMapData().tileSize.y;
		numCellsX = (int)mapData->GetMapData().mapSize.x;
		numCellsY = (int)mapData->GetMapData().mapSize.y;
		mapWidth = numCellsX * cellSizeX;
		mapHeight = numCellsY * cellSizeY;

		obliqueEdgeCost = std::sqrt(std::pow(cellSizeX, 2) + std::pow(cellSizeY, 2));

		cells = new NavGraphCell<NavGraphNode<int>, entity_type>*[numCellsY];
		for (int i = 0; i < numCellsY; ++i)
		{
			cells[i] = new NavGraphCell<NavGraphNode<int>, entity_type>[numCellsX];
			for (int j = 0; j < numCellsX; ++j)
			{
				cells[i][j].cellPosition = glm::vec2(i * cellSizeX + cellSizeX * 0.5, j * cellSizeY + cellSizeY * 0.5);
			}
		}

		navgraph = new Graph();
		CreateGraph();
		graphLayers.push_back(navgraph);

	}
	

	~CellSpacePartition() {}

	//Hierarchical pathfinding//
	///<summary>Create a layer of the abstract graph</summary>
	void CreateGraphLayer(int mapDivision = 2);
	int InsertNodeAtLayer(int layer, glm::vec2 sourcePos);
	void DeleteTemporaryNodesAndEdges();
	int GetNumLayers();

	Cluster<entity_type>* GetCluster(glm::vec2 pos, int layer);

	Graph& GetLayer(int l) const { return *graphLayers[l]; }

	void InsertEntity(entity_type& entity)
	{
		glm::ivec2 gridPos = glm::vec2(entity.GetComponent<Common::TransformComponent>()->GetPosition()) / glm::vec2(cellSizeX, cellSizeY);
		cells[gridPos.x][gridPos.y].entityList.push_front(&entity);

	}


	void UpdateEntity(entity_type& entity, const glm::vec2& oldPos)
	{
		glm::ivec2 newGridPos = glm::vec2(std::fabs(entity.GetComponent<Common::TransformComponent>()->GetPosition().x), std::fabs(entity.GetComponent<Common::TransformComponent>()->GetPosition().y)) / glm::vec2(cellSizeX, cellSizeY);
		glm::ivec2 oldGridPos = glm::vec2(std::fabs(oldPos.x), std::fabs(oldPos.y)) / glm::vec2(cellSizeX, cellSizeY);

		if (oldGridPos != newGridPos)
		{
			cells[oldGridPos.x][oldGridPos.y].entityList.remove(&entity);
			cells[newGridPos.x][newGridPos.y].entityList.push_front(&entity);
		}

	}


	void RemoveEntity(entity_type& entity)
	{
		glm::ivec2 gridPos = glm::vec2(entity.GetComponent<Common::TransformComponent>()->GetPosition()) / glm::vec2(cellSizeX, cellSizeY);
		cells[gridPos.x][gridPos.y].entityList.remove(&entity);

	}


	void CalculateNeighbors(glm::vec2 position, double radius)
	{
		std::vector<entity_type*>::iterator neighbor = neighbors.begin();

		glm::ivec2 gridPos = position / glm::vec2(cellSizeX, cellSizeY);

		for (int y = gridPos.y - 1; y <= gridPos.y + 1; ++y)
		{
			for (int x = gridPos.x - 1; x <= gridPos.x + 1; ++x)
			{
				if (x >= 0 && y >= 0 && x < numCellsX && y < numCellsY && !cells[x][y].entityList.empty())
				{
					for (std::list<entity_type*>::iterator entity = cells[x][y].entityList.begin(), end = cells[x][y].entityList.end(); entity != end; ++entity)
					{
						double distSQ = std::pow(position.x - (*entity)->GetComponent<Common::TransformComponent>()->GetPosition().x, 2.) + std::pow(position.y - (*entity)->GetComponent<Common::TransformComponent>()->GetPosition().y, 2.);
						if (distSQ <= std::pow(radius, 2.))
						{
							*neighbor++ = *entity;
						}
					}
				}
			}
		}

		*neighbor = 0;

	}


	void CalculateNodeNeighbors(glm::vec2 position, double radius)
	{
		std::vector<NavGraphNode<int>*>::iterator node = nodeNeighbors.begin();

		glm::ivec2 gridPos = position / glm::vec2(cellSizeX, cellSizeY);

		for (int y = gridPos.y - 1; y <= gridPos.y + 1; ++y)
		{
			for (int x = gridPos.x - 1; x <= gridPos.x + 1; ++x)
			{
				if (x > 0 && y > 0 && x < numCellsX && y < numCellsY && !cells[y][x].entityList.empty())
				{
					for (std::list<NavGraphNode<int>*>::iterator entity = cells[y][x].nodeList.begin(), end = cells[y][x].nodeList.end(); entity != end; ++entity)
					{
						double distSQ = std::pow((*entity)->GetNodePosition().x, 2.) + std::pow((*entity)->GetNodePosition().y, 2.);
						if (distSQ <= std::pow(radius, 2.))
						{
							*node++ = *entity;
						}
					}
				}
			}
		}

		*node = 0;

	}


	glm::vec2 FindClosestResource(glm::vec2 source);
	
	void AddObstacleAtPosition(glm::vec2 pos);
	void RemoveObstacle(int x, int y);

	glm::vec2 GetCellSize() const { return glm::vec2(cellSizeX, cellSizeY); }
	Graph& GetNavGraph() { return *navgraph; };


	int GetClosestNodeToPosition(glm::vec2 pos)
	{
		glm::ivec2 cell(pos.x / cellSizeX, pos.y / cellSizeY);
		if (cells[cell.x][cell.y].nodeIndex >= 0)
		{
			return cells[cell.x][cell.y].nodeIndex;
		}

		CalculateNodeNeighbors(pos, cellSizeX);
		
		int index = -1;
		double minDistance = (std::numeric_limits<double>::max)();
		
		for(size_t i = 0, size = nodeNeighbors.size(); i != size; ++i)
		{
			glm::vec2 vDist = pos - nodeNeighbors[i]->GetNodePosition();
			double distance = (std::pow(vDist.x, 2.) + std::pow(vDist.y, 2.));
			if (minDistance < distance)
			{
				minDistance = distance;
				index = nodeNeighbors[i]->GetIndex();
			}
		}

		return index;

	}


	int GetClosestAbstractNodeToPosition(glm::vec2 pos)
	{
		glm::ivec2 ClusterPos = pos / glm::vec2(clusterWidth, clusterHeight);

		int index = -1;
		double minDistance = (std::numeric_limits<double>::max)();
		for (auto i = clusterGrid[ClusterPos.x][ClusterPos.y].clusterNodes.begin(); i != clusterGrid[ClusterPos.x][ClusterPos.y] - clusterNodes.end(); ++i)
		{
			glm::vec2 direction = (*i)->GetNodePosition() - pos;
			double distance = glm::length(direction);
			if (distance < minDistance)
			{
				minDistance = distance;
				index = (*i)->GetIndex();
			}
		}

		return index;

	}


	bool ThetaLineOfSight(glm::vec2 source, glm::vec2 target)
	{
		glm::ivec2 currCell = source / GetCellSize();
		glm::ivec2 endCell = target / GetCellSize();
		glm::ivec2 distance = endCell - currCell;
		glm::ivec2 s;

		int f = 0;

		if (distance.y < 0)
		{
			distance.y = -distance.y;
			s.y = -1;
		}
		else
		{
			s.y = 1;
		}

		if (distance.x < 0)
		{
			distance.x = -distance.x;
			s.x = -1;
		}
		else
		{
			s.x = 1;
		}

		if (distance.x >= distance.y)
		{
			while (currCell.x != endCell.x)
			{
				f += distance.y;

				int x = currCell.x + (s.x - 1) / 2;
				int y = currCell.y + (s.y - 1) / 2;

				if (f >= distance.x)
				{
					if (cells[x][y].obstacle)
					{
						return false;
					}

					currCell.y += s.y;
					f -= distance.x;					

					y = currCell.y + (s.y - 1) / 2;
				}

				if (f != 0 && cells[x][y].obstacle)
				{
					return false;
				}

				if (distance.y == 0 && cells[x][currCell.y].obstacle && cells[x][currCell.y - 1].obstacle && cells[x][currCell.y + 1].obstacle)
				{
					return false;
				}

				currCell.x += s.x;
			}
		}
		else
		{
			while (currCell.y != endCell.y)
			{
				f += distance.x;

				int x = currCell.x + (s.x - 1) / 2;
				int y = currCell.y + (s.y - 1) / 2;

				if (f >= distance.y)
				{
					if (cells[x][y].obstacle)
					{
						return false;
					}

					f -= distance.y;
					currCell.x += s.x;

					x = currCell.x + (s.x - 1) / 2;
				}

				if (f != 0 && cells[x][y].obstacle)
				{
					return false;
				}

				if (distance.x == 0 && cells[currCell.x][y].obstacle && cells[currCell.x - 1][y].obstacle && cells[currCell.x + 1][y].obstacle)
				{
					return false;
				}
				currCell.y += s.y;
			}
		}

		return true;

	}


	bool AreOnSight(glm::vec2 source, glm::vec2 target)
	{
		glm::ivec2 startCell = source / GetCellSize();
		glm::ivec2 endCell = target / GetCellSize();
		glm::ivec2 distance = endCell - startCell;
		glm::ivec2 absDistance = glm::abs(distance);

		//middle point
		glm::ivec2 s = 2 * glm::ivec2(absDistance.y - absDistance.x, absDistance.x - absDistance.y);
		glm::ivec2 currCell;

		int	f = 0;

		if (absDistance.x > absDistance.y)
		{
			if (distance.x >= 0)
			{
				currCell = startCell;
				f = endCell.x;
			}
			else
			{
				currCell = endCell;
				f = startCell.x;
			}

			if (cells[currCell.x][currCell.y].obstacle)
			{
				return false;
			}

			for (unsigned int i = 0; currCell.x < f; ++i)
			{
				currCell.x += 1;

				//if line above middle point
				if (s.x < 0)
				{
					//calculate middle point for the next step
					s.x += 2 * (absDistance.y + 1);
				}
				else
				{
					//move algorithm up or down depending on the relative position of the target
					if ((distance.x < 0 && distance.y < 0) || (distance.x > 0 && distance.y > 0))
					{
						currCell.y += 1;
					}
					else
					{
						currCell.y -= 1;
					}

					//calculate middle point for the next step
					s.x += 2 * (absDistance.y - absDistance.x);
				}

				if (cells[currCell.x][currCell.y].obstacle)
				{
					return false;
				}
			}
		}
		else
		{
			if (distance.y >= 0)
			{
				currCell = startCell;
				f = endCell.y;
			}
			else
			{
				currCell = endCell;
				f = startCell.y;
			}

			if (cells[currCell.x][currCell.y].obstacle)
			{
				return false;
			}

			for (unsigned int i = 0; currCell.y < f; ++i)
			{
				currCell.y += 1;

				//if line right to middle point
				if (s.y < 0)
				{
					//calculate middle point for the next step
					s.y += 2 * (absDistance.x + 1);
				}
				else
				{
					//move algorithm right or left depending on the relative position of the target
					if ((distance.x < 0 && distance.y < 0) || (distance.x > 0 && distance.y > 0))
					{
						currCell.x += 1;
					}
					else
					{
						currCell.x -= 1;
					}

					//calculate middle point for the next step
					s.y += 2 * (absDistance.x - absDistance.y);
				}

				if (cells[currCell.x][currCell.y].obstacle)
				{
					return false;
				}
			}
		}

		return true;

	}


	entity_type* begin()
	{
		entityItr = neighbors.begin();
		if (end())
		{
			return NULL;
		}

		return (*entityItr);

	}


	entity_type* next() 
	{ 
		entityItr++; 
		if (end()) 
		{
			return NULL;
		}

		return (*entityItr);

	}


	bool end() 
	{ 
		return (*entityItr == NULL);

	}

private:

	void CreateGraph();
	void FloodFill(int nodeToExpand);
	void AddNodeToGrid(int x, int y);
	bool SaveGraph();
	void AddEdgesToGrid(int x, int y);

	//Hierarchical pathfinding//
	///<summary>Calculate cluster size, num of clusters, and num of cells per cluster</summary>
	void CalculateLayerData(int mapDivision);
	bool ClustersAreAdjacent(Cluster<entity_type>* c1, Cluster<entity_type>* c2);
	void BuildClusters();
	void StoreClusterData(int x, int y, int id);
	void BuildEntrances();
	void ConvertEntrancesInTransitions();
	int InsertTransitionNode(int nodeIndex, glm::vec2 nodePosition, Cluster<entity_type>* cluster);
	void BuildIntraEdges();

	///<summary>Build the intra-edges of the source/target node</summary>
	void BuildNodeIntraEdges(Cluster<entity_type>* c1, NavGraphNode<int>* node, int layer);

	///<summary>Build only a subset of intra-edges of the source/target node</summary>
	void BuildNecessaryIntraEdges(Cluster<entity_type>* c1, NavGraphNode<int>* node, int layer);

	///<summary>Build entrances between two adjacent clusters
	void BuildAdjacentClustersEntrances(Cluster<entity_type>* c1, Cluster<entity_type>* c2, int axis);

	Cluster<entity_type>* GetCluster(Entrance<entity_type>* e, int cluster = 0);

private:

	Graph *navgraph;
	NavGraphCell<NavGraphNode<int>, entity_type> **cells;

	Graph *layerGraph;
	std::vector<Graph*> graphLayers;

	//store clusters of one layer
	clusterVector clusterGrid;

	//store clusters of different layers
	std::vector<clusterVector> layerClusters;
	std::vector<Entrance<entity_type>*> entranceVector;
	std::vector<TemporaryClusterData> temporaryClusterData;

	//layer information
	double clusterWidth, clusterHeight;
	int numClusterX;
	int numClusterY;
	int numLayerCellX;
	int numLayerCellY;

	std::vector<entity_type*> neighbors;
	typename std::vector<entity_type*>::iterator entityItr;

	std::vector<NavGraphNode<int>*> nodeNeighbors;
	typename std::vector<NavGraphNode<int>>::iterator nodeItr;

	double mapWidth, mapHeight;
	int numCellsX, numCellsY;
	double cellSizeX, cellSizeY;

	int nextIndexNode = 0;

	double obliqueEdgeCost;

	Common::Map *mapData;

};


template <class entity_type>
void CellSpacePartition<entity_type>::CreateGraph()
{
	navgraph = new Graph();
	int numCells = numCellsX * numCellsY;
	for (int i = 0; i != numCells; ++i)
	{
		Graph::NodeType *node = new Graph::NodeType(nextIndexNode++, glm::vec2(0.f, 0.f));
		navgraph->InsertNode(*node);
	}

	nextIndexNode = 0;

	glm::ivec2 node(cells[0][0].cellPosition);

	AddNodeToGrid(node.x / (int)cellSizeX, node.y / (int)cellSizeY);
	FloodFill(0);

}


template <class entity_type>
void CellSpacePartition<entity_type>::FloodFill(int nodeToExpand)
{
	if (nodeToExpand < navgraph->GetNumNodes())
	{
		glm::ivec2 nodeCell = navgraph->GetNode(nodeToExpand++).GetNodePosition() 
												/ glm::vec2(cellSizeX, cellSizeY);

		AddNodeToGrid(nodeCell.x + 1, nodeCell.y);
		AddNodeToGrid(nodeCell.x, nodeCell.y + 1);
		AddNodeToGrid(nodeCell.x - 1, nodeCell.y);
		AddNodeToGrid(nodeCell.x, nodeCell.y - 1);

		AddEdgesToGrid(nodeCell.x, nodeCell.y);

		FloodFill(nodeToExpand);
	}

}


template <class entity_type>
void CellSpacePartition<entity_type>::AddNodeToGrid(int x, int y)
{
	if (x >= 0 && x < numCellsY && y >= 0 && y < numCellsX)
	{
		if (cells[x][y].nodeIndex < 0)
		{
			cells[x][y].nodeIndex = nextIndexNode;
			cells[x][y].nodeList.push_back(&navgraph->GetNode(nextIndexNode));
			navgraph->GetNode(nextIndexNode++).SetNodePosition(cells[x][y].cellPosition);
			navgraph->GetNode(cells[x][y].nodeIndex).SetNodeInfo(mapData->GetMapData().tileData[y][x].resource);
			if (!mapData->GetMapData().tileData[y][x].walkable)
			{
				cells[x][y].obstacle = true;
				navgraph->RemoveNode(cells[x][y].nodeIndex);
				return;
			}
		}
	}

}


template <class entity_type>
bool CellSpacePartition<entity_type>::SaveGraph()
{
	return false;

}


template <class entity_type>
void CellSpacePartition<entity_type>::AddEdgesToGrid(int x, int y)
{
	assert(cells[x][y].nodeIndex >= 0);

	int index = cells[x][y].nodeIndex;
	for (int i = y-1; i < y+2; ++i)
	{
		for (int j = x-1; j < x+2; ++j)
		{
			//if cell not out of border
			if (j >= 0 && j < numCellsY && i >= 0 && i < numCellsX)
			{
				int to = cells[j][i].nodeIndex;
				//if node is valid and it is not the source node
				if (to >= 0 && index != to)
				{
					//vertical/horizontal edge
					if (i == y || j == x)
					{
						if (i != y)
						{
							double cost = mapData->GetMapData().tileData[i][j].movementCost;
							cost = mapData->GetMapData().tileData[y][x].movementCost;
							Graph::EdgeType edge(index, to, cellSizeY + ((mapData->GetMapData().tileData[i][j].movementCost + mapData->GetMapData().tileData[y][x].movementCost) / 2.));
							navgraph->InsertEdge(edge);
						}
						else
						{
							double cost = mapData->GetMapData().tileData[i][j].movementCost;
							cost = mapData->GetMapData().tileData[y][x].movementCost;
							Graph::EdgeType edge(index, to, cellSizeX + ((mapData->GetMapData().tileData[i][j].movementCost + mapData->GetMapData().tileData[y][x].movementCost) / 2.));
							navgraph->InsertEdge(edge);
						}
					}
					//oblique edge
					else
					{
						double cost = mapData->GetMapData().tileData[i][j].movementCost;
						cost = mapData->GetMapData().tileData[y][x].movementCost;
						Graph::EdgeType edge(index, to, obliqueEdgeCost + ((mapData->GetMapData().tileData[i][j].movementCost + mapData->GetMapData().tileData[y][x].movementCost) / 2.));
						navgraph->InsertEdge(edge);
					}
				}
			}
		}
	}

}


template<class entity_type>
void CellSpacePartition<entity_type>::CreateGraphLayer(int numClusters)
{
	CalculateLayerData(numClusters);

	BuildClusters();
	BuildEntrances();

	layerGraph = new Graph();
	ConvertEntrancesInTransitions();

	BuildIntraEdges();

	//store the information and free the vectors for the next abstract graph
	layerClusters.push_back(clusterGrid);
	for (size_t x = 0, size = clusterGrid.size(); x != size; ++x)
	{
		clusterGrid[x].clear();
	}
	clusterGrid.clear();

	for (size_t i = 0, size = entranceVector.size(); i != size; ++i)
	{
		delete entranceVector[i];
	}
	entranceVector.clear();

	graphLayers.push_back(layerGraph);

	temporaryClusterData.resize(graphLayers.size());

}


template<class entity_type>
int CellSpacePartition<entity_type>::InsertNodeAtLayer(int layer, glm::vec2 nodePos)
{
	if (layer == 0)
	{
		nodePos = GetNavGraph().GetNode(GetClosestNodeToPosition(nodePos)).GetNodePosition();
	}

	//if node already present in the abstract graph
	int nodeIndex = GetLayer(layer).IsNodePresent(nodePos);
	if (nodeIndex != -1)
	{
		return nodeIndex;
	}

	//if node is cached
	if (size_t(layer) < temporaryClusterData.size())
	{
		for (unsigned int i = 0; i != temporaryClusterData[layer].nodes.size(); ++i)
		{
			if (nodePos == temporaryClusterData[layer].nodes[i]->GetNodePosition())
			{
				GetLayer(layer).InsertNode(*temporaryClusterData[layer].nodes[i]);
				for (unsigned int j = 0; j != temporaryClusterData[layer].temporaryEdges[i].size(); ++j)
				{
					GetLayer(layer).InsertEdge(*temporaryClusterData[layer].temporaryEdges[i][j]);
				}

				return temporaryClusterData[layer].nodes[i]->GetIndex();
			}
		}
	}

	int sourceIndex = GetClosestNodeToPosition(nodePos);
	NavGraphNode<int> *tempNode = new NavGraphNode<int>(GetLayer(layer).GetNumNodes(), nodePos);

	temporaryClusterData[layer].nodes.push_back(tempNode);
	GetLayer(layer).InsertNode(*tempNode);

	Cluster<entity_type>* cluster = GetCluster(nodePos, layer);

	BuildNecessaryIntraEdges(cluster, tempNode, layer);

	return tempNode->GetIndex();

}


template<class entity_type>
inline void CellSpacePartition<entity_type>::DeleteTemporaryNodesAndEdges()
{
	for (unsigned int i = 0; i != temporaryClusterData.size(); ++i)
	{
		for (unsigned int j = 0; j != temporaryClusterData[i].nodes.size(); ++j)
		{
			delete temporaryClusterData[i].nodes[j];
		}

		for (unsigned int j = 0; j != temporaryClusterData[i].temporaryEdges.size(); ++j)
		{
			for (unsigned int k = 0; k != temporaryClusterData[i].temporaryEdges[j].size(); ++k)
			{
				delete temporaryClusterData[i].temporaryEdges[j][k];
			}
			temporaryClusterData[i].temporaryEdges[j].clear();
		}

		temporaryClusterData[i].nodes.clear();
		temporaryClusterData[i].temporaryEdges.clear();
	}

	temporaryClusterData.clear();
	temporaryClusterData.resize(graphLayers.size());

}


template<class entity_type>
int CellSpacePartition<entity_type>::GetNumLayers()
{
	return (int)graphLayers.size() - 1;

}


template<class entity_type>
Cluster<entity_type>* CellSpacePartition<entity_type>::GetCluster(glm::vec2 nodePos, int layer)
{
	assert(layer > 0 && "CellSpacePartition::GetCluster > invalid layer");

	glm::ivec2 clusterPos = nodePos / layerClusters[layer - 1][0][0].clusterSize;
	Cluster<entity_type>* cluster = &layerClusters[layer - 1][clusterPos.x][clusterPos.y];

	return cluster;

}


template<class entity_type>
void CellSpacePartition<entity_type>::CalculateLayerData(int numClusters)
{
	//clusters size
	clusterWidth = mapWidth / numClusters;
	clusterHeight = mapHeight / numClusters;

	//row/col num of clusters
	numClusterX = int(mapWidth / clusterWidth);
	numClusterY = int(mapHeight / clusterHeight);

	//num cells per cluster
	numLayerCellX = int(clusterWidth / cellSizeX);
	numLayerCellY = int(clusterHeight / cellSizeY);

}


template<class entity_type>
bool CellSpacePartition<entity_type>::ClustersAreAdjacent(Cluster<entity_type>* c1, Cluster<entity_type>* c2)
{
	if (c2 != nullptr)
	{
		if (c2->topLeft == c1->topLeft + glm::vec2(c1->clusterSize.x, 0) || c2->topLeft == c1->topLeft + glm::vec2(0, c1->clusterSize.y))
		{
			return true;
		}
	}

	return false;

}


template<class entity_type>
void CellSpacePartition<entity_type>::BuildClusters()
{
	int clusterID = 0;

	clusterGrid.resize(numClusterX);
	for (unsigned int x = 0; x != numClusterX; ++x)
	{
		if (!clusterGrid[x].size())
		{
			clusterGrid[x].resize(numClusterY);
		}

		for (unsigned int y = 0; y != numClusterY; ++y)
		{
			StoreClusterData(x, y, clusterID++);
		}
	}

}


template<class entity_type>
void CellSpacePartition<entity_type>::StoreClusterData(int x, int y, int clusterID)
{
	clusterGrid[x][y] = Cluster<entity_type>();

	clusterGrid[x][y].clusterID = clusterID;
	clusterGrid[x][y].layerID = (int)graphLayers.size();
	clusterGrid[x][y].clusterSize = glm::vec2(clusterWidth, clusterHeight);
	clusterGrid[x][y].topLeft = glm::vec2(x * clusterWidth, y * clusterHeight);
	clusterGrid[x][y].bottomRight = clusterGrid[x][y].topLeft + glm::vec2(clusterWidth, clusterHeight);

	//resize 2D vector of cluster's cells
	clusterGrid[x][y].clusterCells.resize(numLayerCellX);
	for (unsigned int i = 0; i != numLayerCellX; ++i)
	{
		clusterGrid[x][y].clusterCells[i].resize(numLayerCellY);

		//store the cells within the cluster
		for (unsigned int j = 0; j != numLayerCellY; ++j)
		{
			clusterGrid[x][y].clusterCells[i][j] = &cells[i + (x * numLayerCellX)][j + (y * numLayerCellY)];
		}
	}

}


template<class entity_type>
void CellSpacePartition<entity_type>::BuildEntrances()
{
	//create entrances
	for (int x = 0; x != numClusterX; ++x)
	{
		for (int y = 0; y != numClusterY; ++y)
		{
			//cluster under examination
			Cluster<entity_type>* c1 = &clusterGrid[x][y];
			Cluster<entity_type>* c2 = nullptr;
			Cluster<entity_type>* c3 = nullptr;

			//if valid clusters
			if (x < numClusterX - 1)
			{
				c2 = &clusterGrid[x + 1][y];
			}
			if (y < numClusterY - 1)
			{
				c3 = &clusterGrid[x][y + 1];
			}

			BuildAdjacentClustersEntrances(c1, c2, vertical);
			BuildAdjacentClustersEntrances(c1, c3, horizontal);
		}
	}

}


template<class entity_type>
void CellSpacePartition<entity_type>::ConvertEntrancesInTransitions()
{
	int nextNode = 0;

	for (std::vector<Entrance<entity_type>*>::iterator entranceItr = entranceVector.begin(); entranceItr != entranceVector.end();	++entranceItr)
	{
		Cluster<entity_type>* c1 = GetCluster(*entranceItr, 0);
		Cluster<entity_type>* c2 = GetCluster(*entranceItr, 1);

		int lenght = (*entranceItr)->lenght;

		//if entrance is too long, put two "transitions" between the clusters
		if (lenght >= 5)
		{
			//iterate through the first and last entrance cells
			for (int i = 0; i < (*entranceItr)->entranceCells.size(); i += (int)(*entranceItr)->entranceCells.size() - 1)
			{
				int firstIndex = InsertTransitionNode(nextNode, (*entranceItr)->entranceCells[i].first->cellPosition, c1);
				if (firstIndex == nextNode)
				{
					nextNode++;
				}

				int secondIndex = InsertTransitionNode(nextNode, (*entranceItr)->entranceCells[i].second->cellPosition, c2);
				if (secondIndex == nextNode)
				{
					nextNode++;
				}

				//retrieve effective cost of travelling between the nodes
				double edgeCost = navgraph->GetEdge((*entranceItr)->entranceCells[i].first->nodeList.front()->GetIndex(), (*entranceItr)->entranceCells[i].second->nodeList.front()->GetIndex()).GetCost();
				GraphEdge interEdge(firstIndex, secondIndex, edgeCost);

				layerGraph->InsertEdge(interEdge);
			}
		}
		else
		{
			//create one "transition" in the middle cells of the entrance
			//TODO check if there are nodes near to the new node. in case, don't add it to the graph
			int elementPos = (int)(*entranceItr)->entranceCells.size() / 2;

			int firstIndex = InsertTransitionNode(nextNode, (*entranceItr)->entranceCells[elementPos].first->cellPosition, c1);
			if (firstIndex == nextNode)
			{
				nextNode++;
			}

			int secondIndex = InsertTransitionNode(nextNode, (*entranceItr)->entranceCells[elementPos].second->cellPosition, c2);
			if (secondIndex == nextNode)
			{
				nextNode++;
			}

			double edgeCost = navgraph->GetEdge((*entranceItr)->entranceCells[elementPos].first->nodeList.front()->GetIndex(), (*entranceItr)->entranceCells[elementPos].second->nodeList.front()->GetIndex()).GetCost();
			GraphEdge interEdge(firstIndex, secondIndex, edgeCost);

			layerGraph->InsertEdge(interEdge);
		}
	}

}


template<class entity_type>
int CellSpacePartition<entity_type>::InsertTransitionNode(int nodeIndex, glm::vec2 nodePosition, Cluster<entity_type>* cluster)
{
	NavGraphNode<int> *node = new NavGraphNode<int>(nodeIndex, nodePosition);
	int firstIndex = layerGraph->IsNodePresent(node->GetNodePosition());

	//if there is a node in position
	if (firstIndex != -1)
	{
		return firstIndex;
	}
	else
	{
		cluster->clusterNodes.push_back(node);
		layerGraph->InsertNode(*node);
		return nodeIndex;
	}

}


template<class entity_type>
void CellSpacePartition<entity_type>::BuildIntraEdges()
{
	for (unsigned int x = 0; x != numClusterX; ++x)
	{
		for (unsigned int y = 0; y != numClusterY; ++y)
		{
			Cluster<entity_type>* cluster = &clusterGrid[x][y];

			//Build intra-edges
			for (unsigned int i = 0; i != cluster->clusterNodes.size(); ++i)
			{
				int sourceNode = GetClosestNodeToPosition(cluster->clusterNodes[i]->GetNodePosition());
				for (unsigned int j = 0; j != cluster->clusterNodes.size(); ++j)
				{
					if (i != j)
					{
						int targetNode = GetClosestNodeToPosition(cluster->clusterNodes[j]->GetNodePosition());

						AStar<Graph> search(GetNavGraph(), GetCellSize(), cluster->topLeft, cluster->bottomRight, sourceNode, targetNode);
						if (search.HasFoundTarget())
						{
							GraphEdge edge(cluster->clusterNodes[i]->GetIndex(), cluster->clusterNodes[j]->GetIndex(), search.GetTotalCostToTarget());
							layerGraph->InsertEdge(edge);
						}
					}
				}
			}

			//store the nodes that will need to be searched by the source/target node
			cluster->nodeToSearch.push_back(0);
			for (unsigned int j = 1; j != cluster->clusterNodes.size(); ++j)
			{
				if (!layerGraph->IsEdgePresent(cluster->clusterNodes[0]->GetIndex(), cluster->clusterNodes[j]->GetIndex()))
				{
					bool nodeAlreadyNoted = false;
					for (unsigned int i = 1; i != cluster->nodeToSearch.size(); ++i)
					{
						if (layerGraph->IsEdgePresent(cluster->clusterNodes[j]->GetIndex(), cluster->clusterNodes[cluster->nodeToSearch[i]]->GetIndex()))
						{
							nodeAlreadyNoted = true;
						}
					}
					if (!nodeAlreadyNoted)
					{
						cluster->nodeToSearch.push_back(j);
					}
				}
			}
		}
	}

}


template<class entity_type>
void CellSpacePartition<entity_type>::BuildNodeIntraEdges(Cluster<entity_type>* c1, NavGraphNode<int>* node, int layer)
{
	std::vector<GraphEdge*> edges;
	int sourceNode = GetClosestNodeToPosition(node->GetNodePosition());


	for (unsigned int j = 0; j != c1->clusterNodes.size(); ++j)
	{
		int targetNode = GetClosestNodeToPosition(c1->clusterNodes[j]->GetNodePosition());
		if (AreOnSight(node->GetNodePosition(), c1->clusterNodes[j]->GetNodePosition()))
		{
			double edgeCost = glm::length(node->GetNodePosition() - c1->clusterNodes[j]->GetNodePosition());
			GraphEdge *edge = new GraphEdge(node->GetIndex(), c1->clusterNodes[j]->GetIndex(), edgeCost);
			GetLayer(layer).InsertEdge(*edge);
			edges.push_back(edge);
		}
		else
		{
			AStar<Graph> search(GetNavGraph(), GetCellSize(), c1->topLeft, c1->bottomRight, sourceNode, targetNode);
			if (search.HasFoundTarget())
			{
				GraphEdge *edge = new GraphEdge(node->GetIndex(), c1->clusterNodes[j]->GetIndex(), search.GetTotalCostToTarget());
				GetLayer(layer).InsertEdge(*edge);
				edges.push_back(edge);
			}
		}
	}

	temporaryClusterData[layer].temporaryEdges.push_back(edges);

}


template<class entity_type>
void CellSpacePartition<entity_type>::BuildNecessaryIntraEdges(Cluster<entity_type>* c1, NavGraphNode<int>* node, int layer)
{
	std::vector<GraphEdge*> edges;
	int sourceNode = GetClosestNodeToPosition(node->GetNodePosition());

	for (unsigned int j = 0; j != c1->nodeToSearch.size(); ++j)
	{
		if (AreOnSight(node->GetNodePosition(), c1->clusterNodes[c1->nodeToSearch[j]]->GetNodePosition()))
		{
			double edgeCost = glm::length(node->GetNodePosition() - c1->clusterNodes[c1->nodeToSearch[j]]->GetNodePosition());
			GraphEdge *edge = new GraphEdge(node->GetIndex(), c1->clusterNodes[c1->nodeToSearch[j]]->GetIndex(), edgeCost);
			GetLayer(layer).InsertEdge(*edge);
			edges.push_back(edge);
		}
		else
		{
			int targetNode = GetClosestNodeToPosition(c1->clusterNodes[c1->nodeToSearch[j]]->GetNodePosition());
			AStar<Graph> search(GetNavGraph(), GetCellSize(), c1->topLeft, c1->bottomRight, sourceNode, targetNode);
			if (search.HasFoundTarget())
			{
				GraphEdge *edge = new GraphEdge(node->GetIndex(), c1->clusterNodes[c1->nodeToSearch[j]]->GetIndex(), search.GetTotalCostToTarget());
				GetLayer(layer).InsertEdge(*edge);
				edges.push_back(edge);
			}
		}
	}

	temporaryClusterData[layer].temporaryEdges.push_back(edges);

}


template<class entity_type>
void CellSpacePartition<entity_type>::BuildAdjacentClustersEntrances(Cluster<entity_type>* c1, Cluster<entity_type>* c2, int axis)
{
	assert(!(axis != horizontal && axis != vertical) && "CellSpacePartition::BuildAdjacentClustersEntrances > axis not set correctly");

	//invalid cluster
	if (c2 == nullptr)
	{
		return;
	}

	Entrance<entity_type>* entrance = new Entrance<entity_type>();

	entrance->firstCluster = c1->clusterID;
	entrance->secondCluster = c2->clusterID;

	int constant;

	//create vertical/horizontal entrance
	switch (axis)
	{
	case vertical:
		//TODO change .size() with member variable
		constant = (int)c1->clusterCells.size() - 1;
		assert(constant >= 0 && "CellSpacePartition::BuildAdjacentClustersEntrances > cluster does not have any cell");

		entrance->vertical = 1;
		entrance->horizontal = 0;

		//expand entrance along the clusters border line
		//change to remove "clusterCells"
		for (unsigned int y = 0; y != c1->clusterCells.size(); ++y)
		{
			if (!c1->clusterCells[constant][y]->obstacle && !c2->clusterCells[0][y]->obstacle)
			{
				std::pair<NavGraphCell<NavGraphNode<int>, entity_type>*, NavGraphCell<NavGraphNode<int>, entity_type>*> cellsPair(c1->clusterCells[constant][y], c2->clusterCells[0][y]);
				entrance->entranceCells.push_back(cellsPair);
				entrance->lenght++;
			}
			else if (!entrance->entranceCells.empty())
			{
				entrance->obstacleFound = true;
				entranceVector.push_back(entrance);
				entrance = new Entrance<entity_type>();
				entrance->firstCluster = c1->clusterID;
				entrance->secondCluster = c2->clusterID;
				entrance->vertical = 1;
				entrance->horizontal = 0;
			}
		}

		break;

	case horizontal:
		//TODO change .size() with member variable
		constant = (int)c1->clusterCells[0].size() - 1;
		assert(constant >= 0 && "CellSpacePartition::BuildAdjacentClustersEntrances > cluster does not have any cell");

		entrance->vertical = 0;
		entrance->horizontal = -1;

		for (unsigned int x = 0; x != c1->clusterCells[0].size(); ++x)
		{
			if (!c1->clusterCells[x][constant]->obstacle && !c2->clusterCells[x][0]->obstacle)
			{
				std::pair<NavGraphCell<NavGraphNode<int>, entity_type>*, NavGraphCell<NavGraphNode<int>, entity_type>*> cellsPair(c1->clusterCells[x][constant], c2->clusterCells[x][0]);
				entrance->entranceCells.push_back(cellsPair);
				entrance->lenght++;
			}
			else if (!entrance->entranceCells.empty())
			{
				entrance->obstacleFound = true;
				entranceVector.push_back(entrance);
				entrance = new Entrance<entity_type>();
				entrance->firstCluster = c1->clusterID;
				entrance->secondCluster = c2->clusterID;
				entrance->vertical = 0;
				entrance->horizontal = -1;
			}
		}

		break;


	default:
		break;
	}

	//store the open entrance
	if (!entrance->entranceCells.empty())
	{
		entranceVector.push_back(entrance);
	}

}


template<class entity_type>
Cluster<entity_type>* CellSpacePartition<entity_type>::GetCluster(Entrance<entity_type>* e, int cluster)
{
	for (size_t x = 0, size = clusterGrid.size(); x != size; ++x)
	{
		for (size_t y = 0, size = clusterGrid[x].size(); y != size; ++y)
		{
			//if first cluster of the entrance
			//TODO change 0 with an enum
			if (cluster == 0)
			{
				if (clusterGrid[x][y].clusterID == e->firstCluster)
				{
					return &clusterGrid[x][y];
				}
			}
			else
			{
				if (clusterGrid[x][y].clusterID == e->secondCluster)
				{
					return &clusterGrid[x][y];
				}
			}
		}
	}

	return NULL;

}


template<class entity_type>
glm::vec2 CellSpacePartition<entity_type>::FindClosestResource(glm::vec2 source)
{
	float distance = std::numeric_limits<float>::max();
	glm::vec2 target = glm::vec2();
	for (size_t i = 0, numNodes = navgraph->GetNumNodes(); i != numNodes; ++i)
	{
		if (navgraph->GetNode(i).GetIndex() != -1 && navgraph->GetNode(i).GetNodeInfo() == 10)
		{
			float actualDistance = glm::length(source - navgraph->GetNode(i).GetNodePosition());
			if (actualDistance < distance)
			{
				distance = actualDistance;
				target = navgraph->GetNode(i).GetNodePosition();
			}
		}
	}

	return target;

}


template <class entity_type>
void CellSpacePartition<entity_type>::AddObstacleAtPosition(glm::vec2 pos)
{
	int x = pos.x / cellSizeX;
	int y = pos.y / cellSizeY;

	if (!cells[x][y].obstacle)
	{
		cells[x][y].obstacle = true;
		cells[x][y].entityList.clear();
		navgraph->RemoveNode(cells[x][y].nodeIndex);
	}

}


template <class entity_type>
void CellSpacePartition<entity_type>::RemoveObstacle(int x, int y)
{
	if (cells[x][y].obstacle)
	{
		cells[x][y].obstacle = false;
		if (navgraph->GetNode(cells[x][y].nodeIndex).GetIndex() == -1)
		{
			navgraph->GetNode(cells[x][y].nodeIndex).SetIndex(cells[x][y].nodeIndex);
			AddEdgesToGrid(x, y);
		}
		else
		{
			Graph::NodeType node(nextIndexNode++, cells[x][y].cellPosition);
			navgraph->InsertNode(node);
			AddEdgesToGrid(x, y);
		}
	}

}
