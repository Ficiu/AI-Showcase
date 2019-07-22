#pragma once

/**
 *	GOAPEdge.h
 *
 *	Class to store the information for the GOAP edge
 *
 */

#include "GOAPAction.h"
#include "WorldState.h"


namespace goap 
{

    struct GOAPEdge 
    {

		GOAPEdge();
		GOAPEdge(const WorldState state, int g, int h, int parent_id, const GOAPAction* action);

		int FCost() const
		{
			return actual_cost + heuristic_cost;

		}

        static int last_id;           //static int to assign the ID to the nodes - unique IDs to nodes

        WorldState ws;                //The temporary state of the world at this node.
        int pointed_node_id;         
        int parent_node_id;          
        int actual_cost;             
        int heuristic_cost;
        const GOAPAction* action;     //The action associated to this edge

    };

    bool operator<(const GOAPEdge& lhs, const GOAPEdge& rhs);

}