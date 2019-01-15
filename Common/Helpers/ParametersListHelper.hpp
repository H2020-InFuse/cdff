/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ParametersListHelper.hpp
 * @date 01/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Helpers
 *
 *  The ParametersListHelper is an high level wrapper for the yamlcpp library. It offers common method for the extraction of DFN parameters from a configuration file.
 *
 * @{
 */

#ifndef PARAMETERS_LIST_HELPER_HPP
#define PARAMETERS_LIST_HELPER_HPP

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
#include "ParameterHelper.hpp"

namespace Helpers
{
/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class ParametersListHelper
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		ParametersListHelper();
		~ParametersListHelper();

		template <typename UsageType, typename HelperType = ParameterHelper<UsageType, UsageType> >
		void AddParameter(const std::string& groupName, const std::string& parameterName, UsageType& boundVariable, const UsageType& defaultValue)
			{
			HelperType* helper = new HelperType(parameterName, boundVariable, defaultValue);
			AddParameterHelper(groupName, helper);
			}

		void ReadFile(std::string configurationFilePath);

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
		struct ParametersGroup
			{
			std::string groupName;
			std::vector<ParameterHelperInterface*> parametersList;
			};

		std::vector<ParametersGroup> groupsList;


		void ReadGroup(const YAML::Node& configurationNode);
		ParametersGroup* GetGroup(const std::string& groupName);

		void AddParameterHelper(const std::string& groupName, ParameterHelperInterface* helper);
		std::string Print();
	};

}
#endif
/* ParametersListHelper.hpp */
/** @} */
