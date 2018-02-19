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
ParametersInterface::ParametersInterface(std::string applicationName)
	{
	this->applicationName = applicationName;
	}

ParametersInterface::~ParametersInterface()
	{

	}

void ParametersInterface::AddParameter(std::string groupName, std::string name, int defaultValue, int maxValue)
	{
	ASSERT(defaultValue >= 0 && defaultValue <= maxValue, "Parameter interface error: default value not in range [0, maxValue]");

	Parameter newParameter;
	newParameter.name = name;
	newParameter.maxValue = maxValue;
	newParameter.value = defaultValue;
	newParameter.type = (maxValue <= 1) ? BOOL_TYPE : INT_TYPE;
	newParameter.minValue = 0;

	AddParameter(groupName, newParameter);
	}

void ParametersInterface::AddParameter(std::string groupName, std::string name, double defaultValue, double maxValue, double resolution)
	{
	ASSERT(defaultValue >= 0 && defaultValue <= maxValue, "Parameter interface error: default value not in range [0, maxValue]");
	ASSERT( std::abs(resolution) >= std::numeric_limits<double>::epsilon(), "Parameter interface error: resolution of real value higher than maximum machine resolution");

	Parameter newParameter;
	newParameter.name = name;
	newParameter.type = (resolution <= 1e-7) ? DOUBLE_TYPE : FLOAT_TYPE;
	newParameter.maxValue = (int) (maxValue / resolution);
	newParameter.value = (int) (defaultValue/ resolution);	
	newParameter.resolution = resolution;	
	newParameter.minValue = 0; 

	AddParameter(groupName, newParameter);
	}

void ParametersInterface::AddSignedParameter(std::string groupName, std::string name, double defaultValue, double maxValue, double resolution)
	{
	ASSERT(defaultValue >= -maxValue && defaultValue <= maxValue, "Parameter interface error: default value not in range [-maxValue, maxValue]");
	ASSERT( std::abs(resolution) >= std::numeric_limits<double>::epsilon(), "Parameter interface error: resolution of real value higher than maximum machine resolution");

	Parameter newParameter;
	newParameter.name = name;
	newParameter.type = (resolution <= 1e-7) ? DOUBLE_TYPE : FLOAT_TYPE;
	newParameter.maxValue = (int) ( (maxValue + maxValue) / resolution);
	newParameter.value = (int) ( (defaultValue + maxValue) / resolution);	
	newParameter.resolution = resolution;	
	newParameter.minValue = -newParameter.maxValue;

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

/*void ParametersInterface::SaveToYaml(std::string filePath)
	{
	XMLDocument document;
	XMLElement* root = document.NewElement(applicationName.c_str());
	document.InsertFirstChild(root);
	for(std::map<std::string, std::vector<Parameter> >::iterator mapEntry = parameterGroupsMap.begin(); mapEntry != parameterGroupsMap.end(); mapEntry++)
		{
		std::string groupName = mapEntry->first;
		std::vector<Parameter>& parametersList = mapEntry->second;
		XMLElement* groupElement = document.NewElement( groupName.c_str() );
		root->InsertEndChild(groupElement);
		for(std::vector<Parameter>::iterator parameter = parametersList.begin(); parameter != parametersList.end(); parameter++)
			{
			XMLElement* parameterElement = document.NewElement(parameter->name.c_str());
			switch(parameter->type)
				{
				case BOOL_TYPE: parameterElement->SetText( (parameter->value == 1) || (parameter->maxValue == 0) ); break;
				case INT_TYPE: parameterElement->SetText( parameter->value ); break;
				case FLOAT_TYPE: parameterElement->SetText( (float)(parameter->value) * (float)(parameter->resolution) ); break;
				case DOUBLE_TYPE: parameterElement->SetText( (double)(parameter->value) * (double)(parameter->resolution) ); break;
				}
			groupElement->InsertEndChild(parameterElement);
			}
		}
	XMLError error =  document.SaveFile(filePath.c_str());
	ASSERT(error == XML_SUCCESS, "Error saving configuration file");
	}*/

void ParametersInterface::SaveToYaml(std::string filePath)
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
				case INT_TYPE: yamlFile << parameter->value - parameter->minValue; break;
				case FLOAT_TYPE: yamlFile << std::setprecision(7) << (float)(parameter->value - parameter->minValue) * (float)(parameter->resolution); break;
				case DOUBLE_TYPE: yamlFile << std::setprecision(13) << (double)(parameter->value - parameter->minValue) * (double)(parameter->resolution); break;
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
void ParametersInterface::AddParameter(std::string groupName, Parameter parameter)
	{
	std::map<std::string, std::vector<Parameter> >::iterator parametersList = parameterGroupsMap.find(groupName);
	if (parametersList == parameterGroupsMap.end())
		{
		parameterGroupsMap[groupName] = std::vector<Parameter>();	
		}
	parameterGroupsMap[groupName].push_back(parameter);
	}

/** @} */

