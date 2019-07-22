#pragma once

/**
 *	GOAPPlanner.h
 *
 *	Class to execute a variation of A* to find the best actions to reach a given goal.
 *	
 */

#include "GOAPAction.h"
#include "GOAPEdge.h"
#include "WorldState.h"

#include <vector>


namespace goap 
{

    class GOAPPlanner 
    {

    public:

        GOAPPlanner();

        std::vector<GOAPAction> PlanActions(const WorldState& start, WorldState& goal, const std::vector<GOAPAction*>& actions);

    private:

        std::vector<GOAPEdge> frontier_;  
        std::vector<GOAPEdge> closed_list_;

        std::vector<goap::GOAPEdge>::iterator IsMemberOfOpen(const WorldState& ws);

        void MoveEdgeFromOpenToClose();
        void AddEdgeToOpenList(GOAPEdge&&);
        int CalculateHeuristic(const WorldState& now, const WorldState& goal) const;

    };

}