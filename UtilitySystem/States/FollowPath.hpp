#pragma once

#include "UtilitySystem/StateTemplate.hpp"
#include <algorithm>


class Unit;


template <class entity_type>
class FollowPath : public StateTemplate<entity_type>
{

public:

	~FollowPath() {}

	virtual void Enter(entity_type* enemy) { /*enemy->StartFollowPathAnimation()*/ }
	virtual void Execute(entity_type* enemy);
	virtual void Exit(entity_type* enemy) { /*enemy->EndFollowPathAnimation()*/ }
	virtual void CalculateUtility(entity_type* enemy);

};


template<class entity_type>
void FollowPath<entity_type>::Execute(entity_type * enemy)
{
	enemy->SetCurrentAction(entity_type::follow_path);

	if (!enemy->IsPathCompleted())
	{
		glm::vec2 steeringForce;
		glm::vec2 force = enemy->UseBehaviour().Separation(enemy->GetSquadUnits());
		if (!enemy->UseBehaviour().AccumulateForce(steeringForce, force))
		{
			enemy->SetSteeringForce(steeringForce);
			return;
		}

		force += enemy->UseBehaviour().PathFollowing();
		if (!enemy->UseBehaviour().AccumulateForce(steeringForce, force))
		{
			enemy->SetSteeringForce(steeringForce);
			return;
		}

		force = enemy->UseBehaviour().Cohesion(enemy->GetSquadUnits());
		if (!enemy->UseBehaviour().AccumulateForce(steeringForce, force))
		{
			enemy->SetSteeringForce(steeringForce);
			return;
		}		

		enemy->SetSteeringForce(steeringForce);
	}

}


template<class entity_type>
void FollowPath<entity_type>::CalculateUtility(entity_type * enemy)
{
	int multiplier = 0;
	if (enemy->GetCurrentAction() == Unit::follow_path && enemy->GetDistFromTarget() > 128 && !enemy->IsInDanger())
	{
		multiplier = 1;
	}

	double utility = ((enemy->GetHPFactor() + enemy->GetDistanceFactor() + 1 - enemy->GetDangerFator()) / 3.) * std::min((int)enemy->GetPath().size(), 1) * multiplier;
	SetWeight(utility);

}
