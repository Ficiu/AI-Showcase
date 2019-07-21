#pragma once
#include <list>
#include <assert.h>
#include <glm\glm.hpp>


class Regulator;

template<class entity_type>
class StateTemplate;


template<class entity_type>
class UtilitySystem
{

public:

	typedef entity_type Entity;
	UtilitySystem(Entity* owner) : owner(owner), updateRegulator(3) {}
	virtual ~UtilitySystem() {}

	//calculate the utility of each action and choose the highest utility action
	void SelectAction();

	void InsertState(StateTemplate<Entity>* state) { stateList.push_back(state); }
	int GetSelectedAction() const { return selectedActionID; }
	StateTemplate<Entity>* GetSelectedAction()
	{
		return *std::next(stateList.begin(), selectedActionID);
	}

	bool IsReady() { return updateRegulator.IsReady(); }

private:

	void CalculateActionUtilities();
	void ChooseHighestUtilityAction();

private:

	Entity* owner;
	std::list<StateTemplate<Entity>*> stateList;
	int selectedActionID = 0;
	Regulator updateRegulator;

};

template<class entity_type>
void UtilitySystem<entity_type>::CalculateActionUtilities()
{
	assert(!stateList.empty() && "CalculateActionUtilities: state list empty or nullptr entity");

	for (StateTemplate<Entity>* i : stateList)
	{
		i->CalculateUtility(owner);
	}
}

template<class entity_type>
void UtilitySystem<entity_type>::ChooseHighestUtilityAction()
{
	double maxWeight = 0.f;
	int count = 0;
	selectedActionID = 0;
	for (StateTemplate<Entity>* i : stateList)
	{
		if (maxWeight < i->GetWeight())
		{
			maxWeight = i->GetWeight();
			selectedActionID = count;
		}

		++count;
	}
}

template<class entity_type>
void UtilitySystem<entity_type>::SelectAction()
{
	CalculateActionUtilities();
	ChooseHighestUtilityAction();
}