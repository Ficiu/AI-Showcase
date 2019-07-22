#pragma once

/**
*	Regulator.hpp
*
*	Class to regulate the frequency of update of each AI component 
*
*/

#include <chrono>


///class used to regulate the frequency of update of each AI component (movemement, decision making, pathfinding, ...)
class Regulator
{

public:

	///constructor calculates the frequency of update in milliseconds and set an initial update time with a random time between 0 and 1 second.
	Regulator(int numUpdatesPerSecond)
	{
		if (numUpdatesPerSecond > 0)
		{
			IntervalBetweenUpdates = 1000. / numUpdatesPerSecond;
		}
		else if (numUpdatesPerSecond == 0)
		{
			//component will update each frame
			IntervalBetweenUpdates = 0.;
		}
		else
		{
			//component won't update
			IntervalBetweenUpdates = -1.;
		}

		NextUpdateTime = double(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count());
		NextUpdateTime += rand() * (1. / RAND_MAX) * 1000.;

	}

	///<summary>check if the component can update</summary>
	bool IsReady()
	{
		__int64 currentTime = std::chrono::duration_cast<std::chrono::milliseconds>
			(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
		if (currentTime >= NextUpdateTime)
		{
			NextUpdateTime = currentTime + IntervalBetweenUpdates + (-10. + rand() * (20. / RAND_MAX));
			return true;
		}

		return false;

	}

private:

	///frequency and time of update in milliseconds
	double IntervalBetweenUpdates;
	double NextUpdateTime;

};