/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ParametersInterface.cpp
 * @date 24/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the parameters interface class.
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
#include "ParametersInterface.hpp"
#include <Errors/Assert.hpp>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <iostream>


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ParametersInterface::ParametersInterface(const std::string& applicationName)
	{
	this->applicationName = applicationName;
	}

ParametersInterface::~ParametersInterface()
	{

	}

void ParametersInterface::AddParameter(const std::string& groupName, std::string name, int defaultValue, int maxValue)
	{
	ASSERT(defaultValue >= 0 && defaultValue <= maxValue, "Parameter interface error: default value not in range [0, maxValue]");

	Parameter newParameter;
	newParameter.name = name;
	newParameter.maxValue = maxValue;
	newParameter.value = defaultValue;
	newParameter.type = (maxValue <= 1) ? BOOL_TYPE : INT_TYPE;
	newParameter.displacement = 0;

	AddParameter(groupName, newParameter);
	}

void ParametersInterface::AddParameter(const std::string& groupName, std::string name, double defaultValue, double maxValue, double resolution)
	{
	ASSERT(defaultValue >= 0 && defaultValue <= maxValue, "Parameter interface error: default value not in range [0, maxValue]");
	ASSERT( std::abs(resolution) >= std::numeric_limits<double>::epsilon(), "Parameter interface error: resolution of real value higher than maximum machine resolution");

	Parameter newParameter;
	newParameter.name = name;
	newParameter.type = (resolution <= 1e-7) ? DOUBLE_TYPE : FLOAT_TYPE;
	newParameter.maxValue = (int) (maxValue / resolution);
	newParameter.value = (int) (defaultValue/ resolution);	
	newParameter.resolution = resolution;	
	newParameter.displacement = 0; 

	AddParameter(groupName, newParameter);
	}

void ParametersInterface::AddSignedParameter(const std::string& groupName, std::string name, double defaultValue, double maxValue, double resolution)
	{
	ASSERT(defaultValue >= -maxValue && defaultValue <= maxValue, "Parameter interface error: default value not in range [-maxValue, maxValue]");
	ASSERT( std::abs(resolution) >= std::numeric_limits<double>::epsilon(), "Parameter interface error: resolution of real value higher than maximum machine resolution");

	Parameter newParameter;
	newParameter.name = name;
	newParameter.type = (resolution <= 1e-7) ? DOUBLE_TYPE : FLOAT_TYPE;
	newParameter.maxValue = (int) ( (maxValue + maxValue) / resolution);
	newParameter.value = (int) ( (defaultValue + maxValue) / resolution);	
	newParameter.resolution = resolution;	
	newParameter.displacement = (int) ( (maxValue) / resolution);

	AddParameter(groupName, newParameter);
	}
	

void ParametersInterface::CreateTrackbars()
	{
	for(std::map<std::string, std::vector<Parameter> >::iterator mapEntry = parameterGroupsMap.begin(); mapEntry != parameterGroupsMap.end(); mapEntry++)
		{
		std::string groupName = mapEntry->first;
		std::vector<Parameter>& parametersList = mapEntry->second;
		std::stringstream interfaceName;
		interfaceName << applicationName<<" "<<groupName;

		cv::namedWindow(interfaceName.str(), CV_WINDOW_NORMAL);
		for(std::vector<Parameter>::iterator parameter = parametersList.begin(); parameter != parametersList.end(); parameter++)
			{
			cv::createTrackbar(parameter->name, interfaceName.str(), &(parameter->value), parameter->maxValue, NULL);
			}
		}
	}

void ParametersInterface::SaveToYaml(const std::string& filePath)
	{
	std::ofstream yamlFile( filePath.c_str() );

	std::string carryString = "";
	for(std::map<std::string, std::vector<Parameter> >::iterator mapEntry = parameterGroupsMap.begin(); mapEntry != parameterGroupsMap.end(); mapEntry++)
		{
		std::string groupName = mapEntry->first;
		std::vector<Parameter>& parametersList = mapEntry->second;
		yamlFile<<carryString;
		yamlFile<<"-  Name: "<<groupName;

		for(std::vector<Parameter>::iterator parameter = parametersList.begin(); parameter != parametersList.end(); parameter++)
			{
			yamlFile<<std::endl;
			yamlFile<<"   "<<parameter->name<<": ";
			switch(parameter->type)
				{
				case BOOL_TYPE: yamlFile << ( (parameter->value == 1) || (parameter->maxValue == 0) ? "true" : "false"); break;
				case INT_TYPE: yamlFile << parameter->value - parameter->displacement; break;
				case FLOAT_TYPE: yamlFile << std::setprecision(7) << (float)(parameter->value - parameter->displacement) * (float)(parameter->resolution); break;
				case DOUBLE_TYPE: yamlFile << std::setprecision(13) << (double)(parameter->value - parameter->displacement) * (double)(parameter->resolution); break;
				}
			}

		carryString = "\n";
		}
	
	yamlFile.close();
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void ParametersInterface::AddParameter(const std::string& groupName, Parameter parameter)
	{
	std::map<std::string, std::vector<Parameter> >::iterator parametersList = parameterGroupsMap.find(groupName);
	if (parametersList == parameterGroupsMap.end())
		{
		parameterGroupsMap[groupName] = std::vector<Parameter>();	
		}
	parameterGroupsMap[groupName].push_back(parameter);
	}

/** @} */

