#pragma once

#include "UtilitySystem/StateTemplate.hpp"
#include "RTSCommon/Components/TransformComponent.hpp"


class Unit;
class Regulator;


template<class entity_type>
class Attack : public StateTemplate<entity_type>
{

public:

	~Attack() {}

	virtual void Enter(entity_type* enemy)
	{
		enemy->SetVelocity(glm::vec2());
		enemy->SetSteeringForce(glm::vec2());

		nextAction = new Regulator(2);
	}

	virtual void Execute(entity_type* enemy);

	virtual void Exit(entity_type* enemy) 
	{ 
		enemy->numAttacks = 0; 
		delete nextAction;
	}

	virtual void CalculateUtility(entity_type* enemy);

private:

	bool IsApproachingTarget(entity_type * enemy);

	Regulator* nextAction;

};


template<class entity_type>
void Attack<entity_type>::Execute(entity_type * enemy)
{
	enemy->SetCurrentAction(entity_type::attack);

	if (IsApproachingTarget(enemy)) { return; }
	enemy->SetSteeringForce(glm::vec2());
	if (nextAction->IsReady())
	{
		std::cout << "Attack... ";
		int success = rand() % 3;
		if (success <= 1)
		{
			std::cout << "Success!!!" << std::endl;
			enemy->numAttacks++;
			enemy->ExecuteAttack();
		}
		else
		{
			std::cout << "Epic fail!!!" << std::endl;
		}
	}

}


template<class entity_type>
void Attack<entity_type>::CalculateUtility(entity_type* enemy)
{
	int multiplier = 0;
	if (enemy->GetCurrentAction() == Unit::attack || enemy->IsInDanger())
	{
		multiplier = 1;
	}

	//TODO: Add the threat factor to the equation
	double utility = ((enemy->GetHPFactor() + enemy->GetDangerFator()) / 2.) * multiplier;
	SetWeight(utility);

}


template<class entity_type>
bool Attack<entity_type>::IsApproachingTarget(entity_type * enemy)
{
	glm::vec2 enemyPos = enemy->GetClosestEnemyPosition();
	if (glm::length(glm::vec2(enemy->GetComponent<Common::TransformComponent>()->GetPosition()) - enemyPos) > 32)
	{
		glm::vec2 steeringForce;
		steeringForce = enemy->UseBehaviour().Arrive(enemyPos);
		steeringForce += enemy->UseBehaviour().Separation(enemy->GetSquadUnits());
		enemy->SetSteeringForce(steeringForce);
		return true;
	}

	return false;

}
