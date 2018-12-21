/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PerformanceTestInterface.cpp
 * @date 16/03/2018
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
PerformanceTestInterface::PerformanceTestInterface(const std::string& folderPath, const std::string& baseConfigurationFileName, const std::string& performanceMeasuresFileName) :
	PerformanceTestBase(folderPath, std::vector<std::string>({baseConfigurationFileName}), performanceMeasuresFileName)
	{
	ASSERT(temporaryConfigurationFilePathsList.size() == 1, "Initialization did not work as expected, wrong number");
	}

PerformanceTestInterface::~PerformanceTestInterface()
	{

	}

void PerformanceTestInterface::SetDfpc(CDFF::DFPC::DFPCCommonInterface* dfpc)
	{
	this->dfpc = dfpc;
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void PerformanceTestInterface::Configure()
	{
	ASSERT(temporaryConfigurationFilePathsList.size() == 1, "Configuration error, only one configuration file should be provided for one tested DFN");
	dfpc->setConfigurationFile(temporaryConfigurationFilePathsList.at(0));
	dfpc->setup();
	}

void PerformanceTestInterface::Process()
	{
	ExecuteDfpc();
	}

void PerformanceTestInterface::ExecuteDfpc()
	{
	dfpc->run();
	}

/** @} */
