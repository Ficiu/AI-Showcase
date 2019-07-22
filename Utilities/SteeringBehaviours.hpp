#pragma once

/**
*	SteeringBehaviours.hpp
*
*	Template class to encapsulate steering behaviours for the game entities
*
*/

#include <glm/glm.hpp>
#include <vector>

#include "RTSCommon/Components/TransformComponent.hpp"
#include "RTSGame/Hierarchical AI/Unit.hpp"


template <class AI_agent>
class SteeringBehaviours
{

public:

	SteeringBehaviours(AI_agent *entity, double cellSize);
	~SteeringBehaviours() {}

	glm::vec2 Seek(glm::vec2 targetPos);
	glm::vec2 Flee(glm::vec2 targetPos);
	glm::vec2 Arrive(glm::vec2 targetPos, int deleceration = 1);
	glm::vec2 Wander();
	glm::vec2 CollisionAvoidance(glm::vec2 obstaclePos, double obstacleRadius);
	glm::vec2 PathFollowing();
	glm::vec2 Separation(const std::vector<AI_agent*>& squadMembers);
	glm::vec2 Allignment(const std::vector<AI_agent*>& squadMembers);
	glm::vec2 Cohesion(const std::vector<AI_agent*>& squadMembers);

	//Calculate the amount of force that can still be applied and add that amount to the current force
	bool AccumulateForce(glm::vec2 &currentForce, glm::vec2 forceToAdd);

private:

	double RandInRange(float x = -1.f, float y = 1.f)
	{
		return (x + std::rand() * ((y - x) / RAND_MAX));
	}

private:

	AI_agent* owner;

	float wanderRadius, wanderDistance, wanderJitter;
	glm::vec2 wanderTarget;

	double seekDistance;

	float WallAvoidanceWeight = 10.f;
	float SeparationWeight = 10.f;
	float AllignmentWeight = 5.f;
	float CohesionWeight = 0.5f;
	float WanderWeight = 1.f;
	float ArriveWeight = 1.f;
	float SeekWeight = 0.5f;

};


template<class AI_agent>
SteeringBehaviours<AI_agent>::SteeringBehaviours(AI_agent *entity, double cellSize) : owner(entity), wanderTarget(0.f)
{
	seekDistance = cellSize * 0.5;

}


template<class AI_agent>
glm::vec2 SteeringBehaviours<AI_agent>::Seek(glm::vec2 targetPos)
{
	glm::vec2 desiredVelocity = glm::normalize(targetPos - glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition())) * owner->GetMaxSpeed();
	glm::vec2 steeringForce = desiredVelocity - owner->GetVelocity();

	return steeringForce * SeekWeight;

}


template<class AI_agent>
glm::vec2 SteeringBehaviours<AI_agent>::Flee(glm::vec2 targetPos)
{
	const double distance = glm::pow(100., 2);
	if (glm::length(glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition()) - targetPos) > distance)
	{
		return glm::vec2(0.f);
	}

	glm::vec2 desiredVelocity = glm::normalize(glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition()) - targetPos) * owner->GetMaxSpeed();
	glm::vec2 steeringForce = desiredVelocity - owner->GetVelocity();

	return steeringForce * SeekWeight;

}


template<class AI_agent>
glm::vec2 SteeringBehaviours<AI_agent>::Arrive(glm::vec2 targetPos, int deleceration)
{
	glm::vec2 distVector = targetPos - glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition());
	float distance = glm::length(distVector);
	if (distance > 0)
	{
		float decelerationFactor = 0.3f;
		float ramped_speed = distance / ((float)deleceration * decelerationFactor);
		ramped_speed = glm::min(ramped_speed, owner->GetMaxSpeed());
		glm::vec2 desiredVelocity = distVector * ramped_speed / distance;
		glm::vec2 steeringForce = desiredVelocity - owner->GetVelocity();
		return steeringForce * ArriveWeight;
	}

	return glm::vec2(0.f);

}


template<class AI_agent>
glm::vec2 SteeringBehaviours<AI_agent>::Wander()
{
	wanderTarget += glm::vec2(RandInRange() * wanderJitter, RandInRange * wanderJitter);
	wanderTarget = glm::normalize(wanderTarget) * wanderRadius;
	glm::vec2 circleDistance = glm::length(owner->GetVelocity()) * wanderDistance;
	glm::vec2 steeringForce = wanderTarget + circleDistance;

	return steeringForce * WanderWeight;

}


template<class AI_agent>
glm::vec2 SteeringBehaviours<AI_agent>::CollisionAvoidance(glm::vec2 obstaclePos, double obstacleRadius)
{
	glm::vec2 steeringForce;
	glm::vec2 direction = glm::normalize(owner->GetVelocity());
	const double avoidanceDistance = 5.;

	std::vector<glm::vec2> aheadPositions;
	aheadPositions.push_back(glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition()));
	aheadPositions.push_back(glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition()) + direction * avoidanceDistance * 0.5);
	aheadPositions.push_back(glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition()) + direction * avoidanceDistance);

	std::vector<double> distances;
	for (size_t i = 0, size = aheadPositions.size(); i != size; ++i)
	{
		distances.push_back(glm::length(obstaclePos - aheadPositions[i]));
	}

	double collisionRadius = obstacleRadius + owner->GetRadius();

	if (distances[0] < collisionRadius || distances[1] < collisionRadius || distances[2] < collisionRadius)
	{
		steeringForce = glm::normalize(glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition()) - obstaclePos) / glm::length(obstaclePos - owner->GetComponent<Common::TransformComponent>()->GetPosition());
		
		glm::vec2 perpendicularDirection(direction.y, -direction.x);
		float lateralDistance = glm::dot(perpendicularDirection, (obstaclePos - glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition())));
		if (!lateralDistance)
		{
			lateralDistance = 0.1;
		}

		steeringForce += perpendicularDirection / lateralDistance;
	}

	return steeringForce * WallAvoidanceWeight;

}


template<class AI_agent>
glm::vec2 SteeringBehaviours<AI_agent>::PathFollowing()
{
	double distance = glm::length(glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition()) - owner->GetCurrentWaypoint());
	if (distance < seekDistance)
	{
		owner->SetNextWaypoint();
	}

	if (owner->IsPathCompleted())
	{
		return glm::vec2(0.f);
	}

	if (owner->GetCurrentWaypoint() != owner->GetTargetPosition())
	{
		return Seek(owner->GetCurrentWaypoint());
	}
	else
	{
		return Arrive(owner->GetCurrentWaypoint(), 3);
	}

}


template<class AI_agent>
glm::vec2 SteeringBehaviours<AI_agent>::Separation(const std::vector<AI_agent*>& squadUnits)
{
	glm::vec2 steeringForce;

	for (size_t i = 0, size = squadUnits.size(); i != size; ++i)
	{
		if (squadUnits[i] != owner)
		{
			glm::vec2 fromAgent = glm::vec2(owner->GetComponent<Common::TransformComponent>()->GetPosition() - squadUnits[i]->GetComponent<Common::TransformComponent>()->GetPosition());

			//the closer, the stronger the force
			steeringForce += glm::normalize(fromAgent) / glm::length(fromAgent);
		}
	}

	return (steeringForce * SeparationWeight);

}


template<class AI_agent>
glm::vec2 SteeringBehaviours<AI_agent>::Allignment(const std::vector<AI_agent*>& squadUnits)
{
	glm::vec2 averageHeading;
	unsigned int numNeighbours = 0;

	for (size_t i = 0, size = squadUnits.size(); i != size; ++i)
	{
		if (squadUnits[i] != owner && squadUnits[i]->GetVelocity() != glm::vec2())
		{
			averageHeading += glm::normalize(squadUnits[i]->GetVelocity());
			++numNeighbours;
		}
	}

	if (numNeighbours)
	{
		averageHeading /= (double)numNeighbours;
		if (owner->GetVelocity() != glm::vec2())
		{
			averageHeading -= glm::normalize(owner->GetVelocity());
		}
	}

	return averageHeading * AllignmentWeight;

}


template<class AI_agent>
glm::vec2 SteeringBehaviours<AI_agent>::Cohesion(const std::vector<AI_agent*>& squadUnits)
{
	glm::vec2 squadPosition, steeringForce;
	unsigned int numNeighbours = 0;

	for (size_t i = 0, size = squadUnits.size(); i != size; ++i)
	{
		if (squadUnits[i] != owner)
		{
			squadPosition += glm::vec2(squadUnits[i]->GetComponent<Common::TransformComponent>()->GetPosition());
			++numNeighbours;
		}
	}

	if (numNeighbours)
	{
		squadPosition /= (double)numNeighbours;
		steeringForce = Seek(squadPosition);
	}

	return steeringForce * CohesionWeight;

}


template<class AI_agent>
bool SteeringBehaviours<AI_agent>::AccumulateForce(glm::vec2 & currentForce, glm::vec2 forceToAdd)
{
	float magnitudeSoFar = glm::length(currentForce);
	float magnitudeRemaining = owner->GetMaxForce() - magnitudeSoFar;
	if (magnitudeRemaining <= 0.)
	{
		return false;
	}

	double magnitudeToAdd = glm::length(forceToAdd);
	if (magnitudeToAdd < magnitudeRemaining)
	{
		currentForce += forceToAdd;
	}
	else
	{
		currentForce = glm::normalize(forceToAdd) * magnitudeRemaining;
	}

	return true;

}
