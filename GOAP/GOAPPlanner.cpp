#include "GOAPPlanner.h"

#include <algorithm>
#include <cassert>


goap::GOAPPlanner::GOAPPlanner() = default;


int goap::GOAPPlanner::CalculateHeuristic(const goap::WorldState& now, const goap::WorldState& goal) const 
{
    return goal.DistanceTo(now);

}


void goap::GOAPPlanner::AddEdgeToOpenList(GOAPEdge&& n) 
{
    // insert maintaining sort order
    std::vector<GOAPEdge>::const_iterator it = std::lower_bound(begin(frontier_), end(frontier_), n);
    frontier_.emplace(it, std::move(n));

}


void goap::GOAPPlanner::MoveEdgeFromOpenToClose() 
{
    assert(!frontier_.empty());
    closed_list_.push_back(std::move(frontier_.front()));
    frontier_.erase(frontier_.begin());

}


std::vector<goap::GOAPEdge>::iterator goap::GOAPPlanner::IsMemberOfOpen(const goap::WorldState& ws) 
{
    return std::find_if(begin(frontier_), end(frontier_), [&](const GOAPEdge & n) { return n.ws == ws; });

}


std::vector<goap::GOAPAction> goap::GOAPPlanner::PlanActions(const goap::WorldState& start, goap::WorldState& goal, const std::vector<GOAPAction*>& actions) 
{
    if (goal.MeetsGoal(start))
    {
        return std::vector<GOAPAction>();
    }

    frontier_.clear();
    closed_list_.clear();

    GOAPEdge starting_edge(goal, 0, CalculateHeuristic(goal, start), 0, nullptr);

    frontier_.push_back(std::move(starting_edge));

    while (!frontier_.empty())
    {
        //GOAPEdge with the lowest cost from the edges in the frontier_
        //popAndClose() removes the lowest F cost edge from the frontier_ and pushes it back into the closed list
        MoveEdgeFromOpenToClose();
        GOAPEdge& current(closed_list_.back());

        //check for termination
        if (current.action != nullptr && current.action->OperableOn(start))
        {
            std::vector<GOAPAction> the_plan;
            do 
            {
                the_plan.push_back(*current.action);

                std::vector<GOAPEdge>::iterator itr = std::find_if(begin(closed_list_), end(closed_list_), [&](const GOAPEdge & n) 
                                                      { return n.pointed_node_id == current.parent_node_id; });

                current = *itr;
            } while (current.parent_node_id != 0);

            GOAPEdge::last_id = 0;
            return the_plan;
        }

        // Check if action can lead to current/temporary state
        std::vector<GOAPAction*>::const_iterator potential_action;
        for (potential_action = actions.begin(); potential_action != actions.end(); ++potential_action)
        {
            if ((*potential_action)->LeadTo(current.ws))
            {
                //create new state/node
                const goap::WorldState outcome_state = (*potential_action)->ChangeState(current.ws);

                const std::vector<goap::GOAPEdge>::iterator outcome_node = IsMemberOfOpen(outcome_state);

                //if GOAPEdge is not a member of the frontier_/open list
                if (outcome_node == end(frontier_))
                {
                    //Create new GOAPEdge and add it to the open list
                    GOAPEdge found(outcome_state, current.actual_cost + (*potential_action)->GetCost(), CalculateHeuristic(outcome_state, start), current.pointed_node_id, (*potential_action));
                    //addToOpenList mantains the open list sorted per FCost
                    AddEdgeToOpenList(std::move(found));
                }
                else //if GOAPEdge already a member of the frontier_
                {
                    //if the new found path is better than the recorded one
                    if (current.actual_cost + (*potential_action)->GetCost() < outcome_node->actual_cost)
                    {
                        outcome_node->parent_node_id = current.pointed_node_id;
                        outcome_node->actual_cost = current.actual_cost + (*potential_action)->GetCost();
                        outcome_node->heuristic_cost = CalculateHeuristic(outcome_state, start);
                        outcome_node->action = (*potential_action);

                        //sort the open list for the new found path
                        std::sort(begin(frontier_), end(frontier_));
                    }
                }
            }
        }
    }

    // If there's nothing left to evaluate, then we have no possible path left
    throw std::runtime_error("A* planner could not find a path from start to goal");

}
