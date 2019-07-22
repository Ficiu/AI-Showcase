#pragma once

/**
*	PathPlanner.hpp
*
*	Class to manage the Hierarchical A* algorithm and smooth the final path
*
*/

#include <list>
#include <glm\glm.hpp>


namespace Common
{
	class Entity;
}
template <class entity_type>
class CellSpacePartition;
class Regulator;


///AI agents hold this class, which the Decicion-Making component will use to request paths
class PathPlanner
{

private:

	enum { node_not_found = -1 };

public:

	PathPlanner(Common::Entity* owner, CellSpacePartition<Common::Entity>& map);
	PathPlanner(CellSpacePartition<Common::Entity>& map);
	~PathPlanner() {}

	bool SquadHierarchicalSearch(glm::vec2 source, glm::vec2 target, std::list<glm::vec2>& path);
	bool UnitHierarchicalSearch(glm::vec2 target, std::list<glm::vec2>& path);
	bool FindPathToPosition(glm::vec2 target, std::list<glm::vec2>& path);
	bool FindLeastCostPathToItemType(unsigned int itemType);

private:

	void ConvertIndicesToVectors(std::list<int> indices, std::list<glm::vec2>& path, int layer = 0);
	void PathSmoothing(std::list<glm::vec2>& path);


	Common::Entity* owner;
	CellSpacePartition<Common::Entity>& mapCells;

};
