#pragma once

/**
 *	WorldState.h
 *	
 *	Class to store the current state of the world
 *	
 */

#include <string>
#include <vector>
#include <map>
#include "Data.h"

namespace goap 
{

    struct WorldState 
    {

		WorldState(std::string name_ = "", Data::Ai_Goal* goal_data = nullptr);

		//Set a world state variable, i.e. "enemy dead" -> true
		void SetBoolState(int var_id, bool value);
		bool GetBoolState(int var_id) const;

		//Check if the given state meets the requirements of this state
		bool MeetsGoal(const WorldState& goal_state) const;

		//calculate how many states differ from the given state to calculate the heuristic value
		int DistanceTo(const WorldState& goal_state) const;

		bool operator==(const WorldState& other) const;
		bool operator==(const std::string& name_) const;

        float priority;
        std::string goal_name;
        std::map<int, bool> state_bools; //the states of the world
        double weight;                            //utility_ value of the goal

        std::vector<int> used_factors;

    };

}
