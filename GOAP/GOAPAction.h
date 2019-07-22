#pragma once


/**
 *	GOAPAction.h
 *
 *	Class to hold the Preconditions and Effects of the actions and use them
 *
 */

#include <string>
#include <unordered_map>
#include "WorldState.h"


template<class entity_type>
class StateTemplate;
class Unit;


namespace goap
{

    class GOAPAction
    {

	public:

		virtual ~GOAPAction() = default;
		GOAPAction();
		GOAPAction(std::string name_, int cost);

		//Can this action operate on the given world state?
		bool OperableOn(const goap::WorldState& ws) const;

		//Can this action lead to the given world state?
		bool LeadTo(const WorldState& ws) const;

		//Create a temporary world state and apply the effects of the action
		WorldState ActOn(const WorldState& ws) const;

		//Same as ActOn function but applies the preconditions instead of the effects
		WorldState ChangeState(const WorldState& ws) const;

		void SetPrecondition(const int key, const bool value)
		{
			preconditions_map_[key] = value;

		}

		void SetEffect(const int key, const bool value)
		{
			effects_map_[key] = value;

		}

		int GetCost() const { return action_cost_; }

		std::string GetName() const { return action_name_; }

		void SetActionState(StateTemplate<Unit>* action) { action_state_ = action; }
		StateTemplate<Unit>* GetActionState() const { return action_state_; }

    protected:

        std::string action_name_;
        int action_cost_;

        std::unordered_map<int, bool> preconditions_map_;
        std::unordered_map<int, bool> effects_map_;

        StateTemplate<Unit>* action_state_ = nullptr;

    };

}