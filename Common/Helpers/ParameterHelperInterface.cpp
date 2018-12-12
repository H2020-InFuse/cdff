/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ParameterHelperInterface.cpp
 * @date 01/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Helpers
 * 
 * Implementation of the ParameterHelperInterface class
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
#include "ParameterHelperInterface.hpp"
#include <Errors/Assert.hpp>

namespace Helpers
{
/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ParameterHelperInterface::ParameterHelperInterface(const std::string& parameterName) :
	parameterName(parameterName)
	{

	}

ParameterHelperInterface::~ParameterHelperInterface()
	{

	}

bool ParameterHelperInterface::HasName(const std::string& parameterName) const
	{
	return (this->parameterName == parameterName);
	}

std::string ParameterHelperInterface::GetName() const
	{
	return parameterName;
	}

/* --------------------------------------------------------------------------
 *
 * Protected Member Functions
 *
 * --------------------------------------------------------------------------
 */

}
/** @} */

