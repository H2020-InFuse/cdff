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

	AddParameter(groupName, newParameter);
	}

void ParametersInterface::CreateTrackbars()
	{
	for(std::map<std::string, std::vector<Parameter> >::iterator mapEntry = parameterGroupsMap.begin(); mapEntry != parameterGroupsMap.end(); mapEntry++)
		{
		std::string groupName = mapEntry->first;
		std::vector<Parameter>& parametersList = mapEntry->second;
		for(std::vector<Parameter>::iterator parameter = parametersList.begin(); parameter != parametersList.end(); parameter++)
			{
			cv::createTrackbar(parameter->name, groupName, &(parameter->value), parameter->maxValue, NULL);
			}
		}
	}

void ParametersInterface::SaveToXml(std::string filePath)
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
		cv::namedWindow(groupName, CV_WINDOW_NORMAL);
		parameterGroupsMap[groupName] = std::vector<Parameter>();	
		}
	parameterGroupsMap[groupName].push_back(parameter);
	}

/** @} */

