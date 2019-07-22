#include "GOAPAction.h"
#include "WorldState.h"


goap::GOAPAction::GOAPAction() : action_cost_(0)
{}


goap::GOAPAction::GOAPAction(std::string name, int cost)
{
    action_name_ = name;
    action_cost_ = cost;

}


bool goap::GOAPAction::OperableOn(const WorldState& ws) const 
{
    for (auto & precondition : preconditions_map_)
    {
        if (ws.state_bools.at(precondition.first) != precondition.second)
        {
            return false;
        }
    }

    return true;

}


bool goap::GOAPAction::LeadTo(const WorldState& ws) const
{
    bool success = false;

    for (auto & effect : effects_map_)
    {
        if (ws.state_bools.find(effect.first) != ws.state_bools.end())
        {
            if (ws.state_bools.at(effect.first) != effect.second)
            {
                success = false;
                break;
            }
            else
            {
                success = true;
            }
        }
    }

    return success;

}


goap::WorldState goap::GOAPAction::ActOn(const WorldState& ws) const 
{
    goap::WorldState tmp(ws);
    for (auto & effect : effects_map_)
    {
        tmp.SetBoolState(effect.first, effect.second);
    }

    return tmp;

}


goap::WorldState goap::GOAPAction::ChangeState(const WorldState& ws) const
{
    goap::WorldState tmp(ws);
    for (auto & precondition : preconditions_map_)
    {
        tmp.SetBoolState(precondition.first, precondition.second);
    }

    return tmp;

}
