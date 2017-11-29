/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file XmlHelper.cpp
 * @date 22/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Common
 * 
 * Implementation of the XmlHelper class.
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
#include "XmlHelper.hpp"


namespace Common {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
void XmlHelper::ExtractInt(XMLElement* xmlParent, const char* elementName, int& outputVariable, std::string errorMessage)
	{
	XMLElement* element = xmlParent->FirstChildElement(elementName);
	if (element != NULL)
		{
		XMLError typeError = element->QueryIntText( &(outputVariable) );
		ASSERT(typeError == XML_SUCCESS, errorMessage);
		}
	}

void XmlHelper::ExtractFloat(XMLElement* xmlParent, const char* elementName, float& outputVariable, std::string errorMessage)
	{
	XMLElement* element = xmlParent->FirstChildElement(elementName);
	if (element != NULL)
		{
		XMLError typeError = element->QueryFloatText( &(outputVariable) );
		ASSERT(typeError == XML_SUCCESS, errorMessage);
		}
	}

void XmlHelper::ExtractDouble(XMLElement* xmlParent, const char* elementName, double& outputVariable, std::string errorMessage)
	{
	XMLElement* element = xmlParent->FirstChildElement(elementName);
	if (element != NULL)
		{
		XMLError typeError = element->QueryDoubleText( &(outputVariable) );
		ASSERT(typeError == XML_SUCCESS, errorMessage);
		}
	}

void XmlHelper::ExtractBool(XMLElement* xmlParent, const char* elementName, bool& outputVariable, std::string errorMessage)
	{
	std::string valueString;
	ExtractString(xmlParent, elementName, valueString, errorMessage);
	if (valueString.find("true") != std::string::npos)
		{ 
		outputVariable = true;
		}
	else if (valueString.find("false") != std::string::npos)
		{
		outputVariable = false;
		}
	else
		{
		ASSERT(false, #errorMessage);
		} 
	}

void XmlHelper::ExtractString(XMLElement* xmlParent, const char* elementName, std::string& outputVariable, std::string errorMessage)
	{
	XMLElement* element = xmlParent->FirstChildElement(elementName);
	if (element != NULL)
		{
		const char* stringPointer = element->GetText();
		ASSERT(stringPointer != NULL, errorMessage);
		outputVariable = stringPointer;
		}
	}


}


/** @} */
