/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DFNsIntegrationTestInterface.cpp
 * @date 02/02/2017
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

using namespace CDFF::DFN;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
DFNsIntegrationTestInterface::DFNsIntegrationTestInterface(int buttonWidth, int buttonHeight) 
	: mainInterface("Control Panel", buttonWidth, buttonHeight)
	{
	ButtonsInterface::ButtonStyle process_button_style;
	process_button_style.backgroundColor = cv::Scalar(96, 174, 39); // Green - Nephritis
	process_button_style.textColor = cv::Scalar(255, 255, 255);     // White

	ButtonsInterface::ButtonClickedCallback callback = std::bind(&DFNsIntegrationTestInterface::ProcessCallback, this);
	mainInterface.AddButton("Process", callback, process_button_style);
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
	unsigned dfnIndex = dfnsList.size();

	dfnsList.push_back(dfn);
	dfn->setConfigurationFile(GetDfnFilePath(dfnIndex));

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

void DFNsIntegrationTestInterface::AddSignedParameter(DFNCommonInterface* dfn, std::string groupName, std::string name, double defaultValue, double maxValue, double resolution)
	{
	unsigned dfnIndex = GetDfnIndex(dfn);
	ASSERT(dfnIndex < dfnsList.size(), "DFNsIntegrationTestInterface, you have to add the dfn before you add parameters");
	parametersInterfaceList.at(dfnIndex).AddSignedParameter(groupName, name, defaultValue, maxValue, resolution);
	}

double DFNsIntegrationTestInterface::GetTotalProcessingTimeSeconds()
	{
	return totalProcessingTime;
	}

double DFNsIntegrationTestInterface::GetLastProcessingTimeSeconds(unsigned timeIndex)
	{
	ASSERT(timeIndex < processingTime.size(), "DFNs Integration Test Interface Error: Trying to read a processing time that was not previously set");
	return processingTime.at(timeIndex);
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
	totalProcessingTime = 0;
	processingTime.clear();
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

	clock_t begin = clock();
	while (!IsProcessCompleted())
		{
		//Maybe I can add a State framework in this class, rather the letting the derived class define everything.
		DFNCommonInterface* nextDfn = PrepareNextDfn();

		clock_t beginStep = clock();
		nextDfn->process();
		clock_t endStep = clock();
		double stepTime = double(endStep - beginStep) / CLOCKS_PER_SEC;
		processingTime.push_back(stepTime);

		UpdateState();
		}
	clock_t end = clock();
	totalProcessingTime += double(end - begin) / CLOCKS_PER_SEC;
	

	DisplayResult();
	}



/** @} */
