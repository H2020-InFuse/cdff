/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ParameterHelper.hpp
 * @date 01/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Helpers
 * 
 *  The parameter helper is a yamlcpp convenience high-level wrapper for accessing a single parameter in a yaml DFN configuration file. 
 * 
 * @{
 */

#ifndef PARAMETER_HELPER_HPP
#define PARAMETER_HELPER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "ParameterHelperInterface.hpp"
#include <stdlib.h>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>
#include <Errors/Assert.hpp>

namespace Helpers
{
/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
template <typename UsageType, typename YamlType = UsageType>
class ParameterHelper : public ParameterHelperInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		ParameterHelper(const std::string& parameterName, UsageType& boundVariable, const UsageType& defaultValue);
		~ParameterHelper();

		void Read(const YAML::Node& configurationNode);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		UsageType defaultValue;
		UsageType& value;

		std::string UsageTypeToString();
		virtual UsageType Convert(const YamlType& value);
	};



template <typename UsageType, typename YamlType>
ParameterHelper<UsageType, YamlType>::ParameterHelper(const std::string& parameterName, UsageType& boundVariable, const UsageType& defaultValue) :
	ParameterHelperInterface(parameterName), value(boundVariable)
	{
	this->defaultValue = defaultValue;
	value = this->defaultValue;
	}

template <typename UsageType, typename YamlType>
ParameterHelper<UsageType, YamlType>::~ParameterHelper()
	{

	}

template <typename UsageType, typename YamlType>
void ParameterHelper<UsageType, YamlType>::Read(const YAML::Node& configurationNode)
	{
	try
		{
		value = Convert( configurationNode[parameterName].as<YamlType>() );
		}
	catch (YAML::Exception exception)
		{
		std::stringstream stream;
		stream << "Parameter "<< parameterName <<" is absent or has wrong type. Type must be " << UsageTypeToString() <<". Default value will be used: "<< defaultValue;
		std::string errorString = stream.str();
		WRITE_TO_LOG(errorString, "");
		value = defaultValue;	
		}
	}

template <typename UsageType, typename YamlType>
std::string ParameterHelper<UsageType, YamlType>::UsageTypeToString()
	{
	if (std::is_same<UsageType, bool>::value)
		{
		return "Boolean";
		}
	else if (std::is_same<UsageType, int>::value)
		{
		return "Integer";
		}
	else if (std::is_same<UsageType, float>::value)
		{
		return "Float";
		}
	else if (std::is_same<UsageType, double>::value)
		{
		return "Double";
		}
	else if (std::is_same<UsageType, std::string>::value)
		{
		return "String";
		}
	else
		{
		return "User Defined";
		}
	}

template <typename UsageType, typename YamlType>
UsageType ParameterHelper<UsageType, YamlType>::Convert(const YamlType& value)
	{
	bool sameTypes = std::is_same<UsageType, YamlType>::value;
	ASSERT(sameTypes, "Parameter Error: UsageType and YamlType are not the same, you have to implement the conversion function in a derived class of ParameterHelper.");

	//This code just transform the variable type from YamlType to UsageType.
	//As precondition above, this code is only called when YamlType and UsageType are the same, and it works only if they are the same.
	YamlType valueCopy = value;
	void* pointerToYamlType = static_cast<void*>(&valueCopy);
	UsageType* pointerToUsageType = static_cast<UsageType*>(pointerToYamlType);
	return *pointerToUsageType;
	}


}
#endif
/* ParameterHelper.hpp */
/** @} */

