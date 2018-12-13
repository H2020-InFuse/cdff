/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ParametersListHelper.cpp
 * @date 01/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Helpers
 * 
 * Implementation of the ParametersListHelper class
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
#include "ParametersListHelper.hpp"
#include <Errors/Assert.hpp>

namespace Helpers
{
/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

ParametersListHelper::ParametersListHelper()
	{

	}

ParametersListHelper::~ParametersListHelper()
	{
	for(std::vector<ParametersGroup>::iterator groupIterator = groupsList.begin(); groupIterator != groupsList.end(); ++groupIterator)
		{
		for(std::vector<ParameterHelperInterface*>::iterator helperIterator = groupIterator->parametersList.begin(); helperIterator != groupIterator->parametersList.end(); ++helperIterator)
			{
			delete(*helperIterator);
			}
		}
	}

void ParametersListHelper::ReadFile(std::string configurationFilePath)
	{
	try
		{
		YAML::Node configuration= YAML::LoadFile( configurationFilePath );
		for(unsigned configuationIndex=0; configuationIndex < configuration.size(); configuationIndex++)
			{
			YAML::Node configurationNode = configuration[configuationIndex];
			ReadGroup(configurationNode);
			}
		} 
	catch(YAML::Exception& e) 
		{
    		ASSERT(false, e.what() );
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

void ParametersListHelper::ReadGroup(const YAML::Node& configurationNode)
	{
	std::string nodeName = configurationNode["Name"].as<std::string>();
	ParametersGroup* matchedGroup = GetGroup(nodeName);
	if (matchedGroup == NULL )
		{
		// we do not mind we have more data in the configuration file
		return;
		}

	for(unsigned parameterIndex = 0; parameterIndex < matchedGroup->parametersList.size(); parameterIndex++)
		{
		ParameterHelperInterface* helper = matchedGroup->parametersList.at(parameterIndex);
		helper->Read(configurationNode);
		}
	}

ParametersListHelper::ParametersGroup* ParametersListHelper::GetGroup(const std::string& groupName)
	{
	for(std::vector<ParametersGroup>::iterator iterator = groupsList.begin(); iterator != groupsList.end(); ++iterator)
		{	
		if (iterator->groupName == groupName)
			{
			return &(*iterator);
			}
		}
	return NULL;
	}

void ParametersListHelper::AddParameterHelper(const std::string& groupName, ParameterHelperInterface* helper)
	{
	ParametersGroup* matchedGroup = GetGroup(groupName);

	if (matchedGroup == NULL )
		{
		ParametersGroup newGroup;
		newGroup.groupName = groupName;
		newGroup.parametersList.push_back(helper);
		groupsList.push_back(newGroup);
		}
	else
		{
		matchedGroup->parametersList.push_back(helper);
		}
	}

std::string ParametersListHelper::Print()
	{
	std::stringstream stream;
	
	for(std::vector<ParametersGroup>::iterator groupIterator = groupsList.begin(); groupIterator != groupsList.end(); ++groupIterator)
		{
		for(std::vector<ParameterHelperInterface*>::iterator helperIterator = groupIterator->parametersList.begin(); helperIterator != groupIterator->parametersList.end(); ++helperIterator)
			{
			stream << groupIterator->groupName <<": "<<(*helperIterator)->GetName()<<std::endl;
			}
		}	

	return stream.str();
	}

}
/** @} */

