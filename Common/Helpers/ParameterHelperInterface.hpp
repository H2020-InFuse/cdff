/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ParameterHelperInterface.hpp
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

#ifndef PARAMETER_HELPER_INTERFACE_HPP
#define PARAMETER_HELPER_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <stdlib.h>
#include <vector>
#include <string>
#include <yaml-cpp/yaml.h>

namespace Helpers
{

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class ParameterHelperInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		ParameterHelperInterface(const std::string& parameterName);
		virtual ~ParameterHelperInterface();

		bool HasName(const std::string& parameterName) const;
		std::string GetName() const;

		virtual void Read(const YAML::Node& configurationNode) = 0;

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:
		std::string parameterName;
	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:

	};


}
#endif
/* ParameterHelperInterface.hpp */
/** @} */

