#include "GOAPEdge.h"


int goap::GOAPEdge::last_id = 0;


goap::GOAPEdge::GOAPEdge() : parent_node_id(0)
                            , actual_cost(0)
                            , heuristic_cost(0)
                            , action(nullptr) 
{
    pointed_node_id = ++last_id;

}


goap::GOAPEdge::GOAPEdge(const WorldState state, int g, int h, int parent_id, const GOAPAction* action) :
                ws(state), parent_node_id(parent_id), actual_cost(g), heuristic_cost(h), action(action)
{
    pointed_node_id = ++last_id;

}


bool goap::operator<(const goap::GOAPEdge& lhs, const goap::GOAPEdge& rhs) 
{
    return lhs.FCost() < rhs.FCost();

}
