#pragma once

#include "RTSCommon/AISystem/UtilitySystem/StateTemplate.hpp"
#include "RTSCommon/Components/TransformComponent.hpp"


class Unit;


template<class entity_type>
class RunAway : public StateTemplate<entity_type>
{

public:

	~RunAway() {}

	virtual void Enter(entity_type* enemy) { /*enemy->StartRunAwayAnimation()*/ }
	virtual void Execute(entity_type* enemy);
	virtual void Exit(entity_type* enemy) { /*enemy->EndRunAwayAnimation()*/ }
	virtual void CalculateUtility(entity_type* enemy);

};


template<class entity_type>
void RunAway<entity_type>::Execute(entity_type * enemy)
{
	enemy->SetSteeringForce(enemy->Flee(enemy->GetTargetPos));

}

template<class entity_type>
void RunAway<entity_type>::CalculateUtility(entity_type* enemy)
{
	float utility = ((1 - enemy->GetHPFactor() + enemy->GetDangerFator()) / 2.);
	SetWeight(utility);

}
