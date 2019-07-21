#pragma once

#include <cassert>


template<class entity_type>
class StateTemplate;

//finite state machine
template<class entity_type>
class StateMachine
{
private:
	StateTemplate<entity_type> *currentState;
	StateTemplate<entity_type> *previousState;
	//state with logic duplicated in all the states
	StateTemplate<entity_type> *globalState;

	entity_type* entityOwner;

public:
	//Set the state variables and the owner of the SM
	StateMachine(entity_type* entityOwner) : entityOwner(entityOwner) {}

	//Initialize FSM states
	void SetCurrentState(StateTemplate<entity_type> *state) { currentState = state; }
	void SetPreviousState(StateTemplate<entity_type> *state) { previousState = state; }
	void SetGlobalState(StateTemplate<entity_type> *state) { globalState = state; }

	//Update FSM
	void Update()
	{
		if (globalState)
		{
			globalState->Execute(entityOwner);
		}

		if (currentState)
		{
			currentState->Execute(entityOwner);
		}
	}

	void ChangeState(StateTemplate<entity_type>* state)
	{
		assert(state && "StateMachine::ChangeState trying to change to a null state");

		if (state == currentState)
		{
			return;
		}

		previousState = currentState;
		currentState->Exit(entityOwner);
		
		currentState = state;
		currentState->Enter(entityOwner);
	}

	void RevertToPreviousState()
	{
		ChangeState(previousState);
	}

	StateTemplate<entity_type>* GetCurrentState() const { return currentState; }
	StateTemplate<entity_type>* GetPreviousState() const { return previousState; }
	StateTemplate<entity_type>* GetGlobalState() const { return globalState; }
};