/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Mock.cpp
 * @date 21/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of the main mocking class.
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "Mock.hpp"
#include <Errors/Assert.hpp>

namespace Mocks {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
void Mock::AddBehaviour(const std::string& functionName, const std::string& time, void* value)	
	{
	AddBehaviour(behavioursMap, functionName, time, value);
	if (countersMap.find(functionName) != countersMap.end())
		{
		countersMap[functionName] = 0;
		}
	}

void* Mock::GetBehaviour(const std::string& functionName, unsigned time)
	{
	return GetBehaviour(behavioursMap, functionName, time);
	}

void Mock::AddStaticBehaviour(const std::string& functionName, const std::string& time, void* value)	
	{
	AddBehaviour(staticBehavioursMap, functionName, time, value);
	}

void* Mock::GetStaticBehaviour(const std::string& functionName, unsigned time)
	{
	return GetBehaviour(staticBehavioursMap, functionName, time);
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
std::map<std::string, std::map<std::string, void*> > Mock::staticBehavioursMap;


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

void Mock::AddBehaviour(std::map<std::string, std::map<std::string, void*> >& map, const std::string& functionName, const std::string& time, void* value)	
	{
	std::map<std::string, std::map<std::string, void*> >::iterator functionBehaviourPair = map.find(functionName);
	if (functionBehaviourPair == map.end())
		{
		std::map<std::string, void*> newBehaviour;
		newBehaviour[time] = (void*)value;
		map[functionName] = newBehaviour;
		}	
	else
		{
		std::map<std::string, void*>& behaviour = functionBehaviourPair->second;
		std::map<std::string, void*>::iterator plannedOperation = behaviour.find(time);
		if (plannedOperation == behaviour.end())
			{
			behaviour[time] = (void*)value;
			}
		}
	}	

void* Mock::GetBehaviour(std::map<std::string, std::map<std::string, void*> >& map, const std::string& functionName, unsigned time)
	{
	std::map<std::string, std::map<std::string, void*> >::iterator functionBehaviourPair = map.find(functionName);
	if (functionBehaviourPair == map.end())
		{
		return NULL;
		}
	
	std::map<std::string, void*>& behaviour = functionBehaviourPair->second;
	std::map<std::string, void*>::iterator plannedOperation = behaviour.find("Always");
	if (plannedOperation != behaviour.end())
		{
		return plannedOperation->second;
		}

	std::stringstream stream;
	stream << time;
	std::string string = stream.str();
	plannedOperation = behaviour.find(string);
	if (plannedOperation != behaviour.end())
		{
		return plannedOperation->second;
		}

	return NULL;			
	}


}
/** @} */
