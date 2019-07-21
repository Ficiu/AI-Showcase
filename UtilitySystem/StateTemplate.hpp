#pragma once


//template state base class
template<class entity_type>
class StateTemplate
{
	double weight = 0.f;

public:
	virtual ~StateTemplate() {}

	virtual void Enter(entity_type*) = 0;
	virtual void Execute(entity_type*) = 0;
	virtual void Exit(entity_type*) = 0;
	virtual void CalculateUtility(entity_type*) = 0;

	void SetWeight(double newWeight) { weight = newWeight; }
	double GetWeight() const { return weight; }
};