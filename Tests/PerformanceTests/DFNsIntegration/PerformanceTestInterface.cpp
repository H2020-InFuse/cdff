/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PerformanceTestInterface.cpp
 * @date 22/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the performance test interface class.
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
#include "PerformanceTestInterface.hpp"
#include <Errors/Assert.hpp>


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PerformanceTestInterface::PerformanceTestInterface(const std::string& folderPath, const std::vector<std::string>& baseConfigurationFileNamesList, const std::string& performanceMeasuresFileName)
	: PerformanceTestBase(folderPath, baseConfigurationFileNamesList, performanceMeasuresFileName)
	{

	}

PerformanceTestInterface::~PerformanceTestInterface()
	{

	}

void PerformanceTestInterface::AddDfn(CDFF::DFN::DFNCommonInterface* dfn)
	{
	dfnsList.push_back(dfn);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void PerformanceTestInterface::Configure()
	{
	ASSERT(dfnsList.size() == temporaryConfigurationFilePathsList.size(), "Configuration error, the number of dfns is not the same as the number of configuration files");
	for(unsigned dfnIndex = 0; dfnIndex < dfnsList.size(); dfnIndex++)
		{
		CDFF::DFN::DFNCommonInterface* dfn = dfnsList.at(dfnIndex);
		dfn->setConfigurationFile( temporaryConfigurationFilePathsList.at(dfnIndex) );
		dfn->configure();
		}
	}

void PerformanceTestInterface::Process()
	{
	ExecuteDfns();
	}

/** @} */
