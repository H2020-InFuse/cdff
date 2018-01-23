/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file YamlcppMacro.hpp
 * @date 22/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Common
 * 
 *  This is a collection of Macro for YamlCpp.
 *   
 * @{
 */

#ifndef YAMLCPP_MACRO_HPP
#define YAMLCPP_MACRO_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <yaml-cpp/yaml.h>
#include <Errors/Assert.hpp>

/* --------------------------------------------------------------------------
 *
 * Macros
 *
 * --------------------------------------------------------------------------
 */
#define HANDLE_YAMLCPP_MISSING_KEY(variable, key, defaultValue) \
        catch (YAML::KeyNotFound exception) \
		{ \
		std::stringstream stream; \
		stream << "Parameter "<< key <<" was not set, default value will be used:  " << defaultValue; \
		variable = defaultValue; \
		} \

#define HANDLE_YAMLCPP_BAD_PARAMETER(key, typeString) \
	catch (YAML::Exception exception) \
		{ \
		std::stringstream stream; \
		stream << "Parameter "<< key <<" should be of type " << typeString; \
		PRINT_TO_LOG(stream.str(), ""); \
		throw exception; \
		} \

#define TRY_EXPRESSION(expression) \
	try \
		{ \
		expression; \
		}	

#define YAMLCPP_ASSIGN(variable, type, defaultValue, node, key) \
	{ \
	TRY_EXPRESSION(variable = node[key].as<type>() ) \
	HANDLE_YAMLCPP_MISSING_KEY(variable, key, defaultValue) \
	HANDLE_YAMLCPP_BAD_PARAMETER(key, #type) \
	}	
	
#define YAMLCPP_ASSIGN_WITH_FUNCTION(variable, type, defaultValue, node, key, function) \
	{ \
	TRY_EXPRESSION(variable = function(node[key].as<type>()) ) \
	HANDLE_YAMLCPP_MISSING_KEY(variable, key, defaultValue) \
	HANDLE_YAMLCPP_BAD_PARAMETER(key, #type) \
	}

#define YAMLCPP_DFN_ASSIGN(variable, type, node, key) \
	YAMLCPP_ASSIGN(parameters.variable, type, DEFAULT_PARAMETERS.variable, node, key)

#define YAMLCPP_DFN_ASSIGN_WITH_FUNCTION(variable, type, node, key, function) \
	YAMLCPP_ASSIGN_WITH_FUNCTION(parameters.variable, type, DEFAULT_PARAMETERS.variable, node, key, function)

#endif

/* YamlcppMacro.hpp */
/** @} */
