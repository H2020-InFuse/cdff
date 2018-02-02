/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DFNsIntegrationTestInterface.cpp
 * @date 27/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the dfns integration test interface class.
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
#include "DFNsIntegrationTestInterface.hpp"
#include <Errors/Assert.hpp>
#include <ctime>
#include <fstream>
#include <stdio.h>

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
DFNsIntegrationTestInterface::DFNsIntegrationTestInterface(int buttonWidth, int buttonHeight) 
	: mainInterface("Control Panel", buttonWidth, buttonHeight)
	{
	mainInterface.AddButton("Process", DFNsIntegrationTestInterface::ProcessCallback, this);
	}

DFNsIntegrationTestInterface::~DFNsIntegrationTestInterface()
	{

	}

void DFNsIntegrationTestInterface::Run()
	{
	SetupMocksAndStubs();
	SetupParameters();
	for(unsigned dfnIndex = 0; dfnIndex < dfnsList.size(); dfnIndex++)
		{
		parametersInterfaceList.at(dfnIndex).CreateTrackbars();
		}
	mainInterface.Run();
	}

void DFNsIntegrationTestInterface::AddDFN(DFNCommonInterface* dfn, std::string dfnName)
	{
	unsigned dfnIndex = dfnsList.size() - 1;

	dfnsList.push_back(dfn);
	dfn->setConfigurationFile(GetDfnFilePath(dfnIndex));

	processingTime.push_back(0);
	parametersInterfaceList.push_back( ParametersInterface(dfnName) );
	}

void DFNsIntegrationTestInterface::AddParameter(DFNCommonInterface* dfn, std::string groupName, std::string name, int defaultValue, int maxValue)
	{
	unsigned dfnIndex = GetDfnIndex(dfn);	
	ASSERT(dfnIndex < dfnsList.size(), "DFNsIntegrationTestInterface, you have to add the dfn before you add parameters");
	parametersInterfaceList.at(dfnIndex).AddParameter(groupName, name, defaultValue, maxValue);
	}

void DFNsIntegrationTestInterface::AddParameter(DFNCommonInterface* dfn, std::string groupName, std::string name, double defaultValue, double maxValue, double resolution)
	{
	unsigned dfnIndex = GetDfnIndex(dfn);
	ASSERT(dfnIndex < dfnsList.size(), "DFNsIntegrationTestInterface, you have to add the dfn before you add parameters");
	parametersInterfaceList.at(dfnIndex).AddParameter(groupName, name, defaultValue, maxValue, resolution);
	}

double DFNsIntegrationTestInterface::GetTotalProcessingTimeSeconds()
	{
	double totalProcessingTime = 0;
	for(unsigned dfnIndex = 0; dfnIndex < dfnsList.size(); dfnIndex++)
		{
		totalProcessingTime += processingTime.at(dfnIndex);
		}
	return totalProcessingTime;
	}

double DFNsIntegrationTestInterface::GetLastProcessingTimeSeconds(DFNCommonInterface* dfn)
	{
	unsigned dfnIndex = GetDfnIndex(dfn);
	if (dfnIndex < dfnsList.size() )
		{
		return processingTime.at(dfnIndex);
		}
	return -1;
	}

int DFNsIntegrationTestInterface::GetTotalVirtualMemoryUsedKB()
	{
	FILE* file = fopen("/proc/self/status", "r");
	int result = -1;
	char line[128];

	while (fgets(line, 128, file) != NULL)
		{
		if (strncmp(line, "VmSize:", 7) == 0)
			{
			// This is code for parsing the line and extracting the data.
			result = strlen(line);
    			const char* p = line;
    			while (*p <'0' || *p > '9') 
				{
				p++;
				}
    			line[result-3] = '\0';
    			result = atoi(p);
			}
		}

	fclose(file);
	return result / 1024;
	
	}
/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */

const std::string DFNsIntegrationTestInterface::baseFilePath = "../../tests/ConfigurationFiles/TemporaryGuiTest/ConfDFN";


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void DFNsIntegrationTestInterface::SetupMocksAndStubs()
	{

	}

void DFNsIntegrationTestInterface::SetupParameters()
	{

	}

void DFNsIntegrationTestInterface::DisplayResult()
	{

	}

unsigned DFNsIntegrationTestInterface::GetDfnIndex(DFNCommonInterface* dfn)
	{
	for(unsigned dfnIndex = 0; dfnIndex < dfnsList.size(); dfnIndex++)
		{
		if (dfnsList.at(dfnIndex) == dfn)
			{
			return dfnIndex;
			}
		}
	return dfnsList.size();
	}

void DFNsIntegrationTestInterface::CleanProcessingTime()
	{
	for(unsigned dfnIndex = 0; dfnIndex < dfnsList.size(); dfnIndex++)
		{
		processingTime.at(dfnIndex) = 0;
		}
	}

void DFNsIntegrationTestInterface::ProcessCallback(void* referenceToClass)
	{
	((DFNsIntegrationTestInterface*)referenceToClass)->ProcessCallback();
	}

std::string DFNsIntegrationTestInterface::GetDfnFilePath(unsigned dfnIndex)
	{
	std::stringstream filePath;
	filePath<<baseFilePath<<"_"<<dfnIndex<<".yaml";
	return filePath.str();
	}

void DFNsIntegrationTestInterface::ConfigureDFNs()
	{
	for(unsigned dfnIndex = 0; dfnIndex < dfnsList.size(); dfnIndex++)
		{
		parametersInterfaceList.at(dfnIndex).SaveToYaml( GetDfnFilePath(dfnIndex) );
		dfnsList.at(dfnIndex)->configure();
		}	
	}

void DFNsIntegrationTestInterface::ProcessCallback()
	{
	ConfigureDFNs();
	CleanProcessingTime();
	ResetProcess();

	while (!IsProcessCompleted())
		{
		unsigned dfnIndex = PrepareNextDfn();
		clock_t begin = clock();
		dfnsList.at(dfnIndex)->process();
		clock_t end = clock();
 		processingTime.at(dfnIndex) += double(end - begin) / CLOCKS_PER_SEC;
		}

	DisplayResult();
	}



/** @} */
