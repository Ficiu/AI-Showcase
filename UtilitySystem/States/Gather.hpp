#pragma once

#include "RTSCommon/AISystem/UtilitySystem/StateMachine.hpp"
#include "RTSCommon/Components/TransformComponent.hpp"


class Unit;
class Regulator;


template<class entity_type>
class Gather : public StateTemplate<entity_type>
{

public:

	~Gather() { delete nextAction; }

	virtual void Enter(entity_type* enemy);
	virtual void Execute(entity_type* enemy);
	virtual void Exit(entity_type* enemy);
	virtual void CalculateUtility(entity_type* enemy);

private:

	bool IsApproachingTarget(entity_type *enemy);

	Regulator* nextAction;

};


template<class entity_type>
void Gather<entity_type>::Enter(entity_type * enemy)
{
	nextAction = new Regulator(2);

	/* enemy->StartGatherAnimation()*/

}


template<class entity_type>
void Gather<entity_type>::Execute(entity_type * enemy)
{
	enemy->SetCurrentAction(entity_type::gather);

	if (IsApproachingTarget(enemy)) { return; }
	enemy->SetSteeringForce(glm::vec2());
	if (nextAction->IsReady())
	{
		std::cout << "Extracting resources." << std::endl;
		enemy->AddResource();
	}

}


template<class entity_type>
void Gather<entity_type>::Exit(entity_type * enemy)
{
	delete nextAction;
	/* enemy->EndGatherAnimation()*/

}


template<class entity_type>
void Gather<entity_type>::CalculateUtility(entity_type* enemy)
{
	int multiplier = 0;
	if (enemy->GetCurrentAction() == Unit::gather && enemy->GetDistFromTarget() <= 128)
	{
		multiplier = 1;
	}

	//TODO - Filippo: Add the threat factor to the equation
	double utility = ((enemy->GetHPFactor() + 1 - enemy->GetDangerFator()) / 2.) * multiplier;
	SetWeight(utility);

}


template<class entity_type>
bool Gather<entity_type>::IsApproachingTarget(entity_type * enemy)
{
	if (enemy->GetDistFromTarget() > 32)
	{
		glm::vec2 steeringForce;
		glm::vec2 force = enemy->UseBehaviour().Separation(enemy->GetSquadUnits());
		if (!enemy->UseBehaviour().AccumulateForce(steeringForce, force))
		{
			enemy->SetSteeringForce(steeringForce);
			return true;
		}

		force += enemy->UseBehaviour().Arrive(enemy->GetTargetPosition());
		if (!enemy->UseBehaviour().AccumulateForce(steeringForce, force))
		{
			enemy->SetSteeringForce(steeringForce);
			return true;
		}

		enemy->SetSteeringForce(steeringForce);
		return true;
	}

	return false;

}