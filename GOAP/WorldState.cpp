#include "WorldState.h"
#include <utility>


goap::WorldState::WorldState(std::string name_, Data::Ai_Goal* goal_data) : priority(0.f),
                                                                           goal_name(std::move(name_)),
                                                                           weight(0.)
{
    if (goal_data)
    {
        name_ = goal_data->name;
        for (auto & goal_state : goal_data->goal_states)
        {
            SetBoolState(goal_state->state, goal_state->reached);
        }
    }

}


void goap::WorldState::SetBoolState(int var_id, const bool value)
{
    state_bools[var_id] = value;

}


bool goap::WorldState::GetBoolState(int var_id) const
{
    return state_bools.at(var_id);

}


bool goap::WorldState::operator==(const WorldState& other) const 
{
    return state_bools == other.state_bools;

}


bool goap::WorldState::operator==(const std::string & name_) const
{
    return goal_name == name_;

}


bool goap::WorldState::MeetsGoal(const WorldState& goal_state) const
{
    for (std::map<int, bool>::const_iterator state = goal_state.state_bools.begin(); state != goal_state.state_bools.end(); ++state)
    {
        if (state_bools.find(state->first) != state_bools.end() && state_bools.at(state->first) != state->second)
        {
            return false;
        }
    }

    return true;

}


int goap::WorldState::DistanceTo(const WorldState& goal_state) const 
{
    int result = 0;

    for (std::map<int, bool>::const_iterator state = goal_state.state_bools.begin(); state != goal_state.state_bools.end(); ++state)
    {
        const std::map<int, bool>::const_iterator itr = state_bools.find(state->first);
        if (itr->second != state->second)
        {
            ++result;
        }
    }

    return result;

}