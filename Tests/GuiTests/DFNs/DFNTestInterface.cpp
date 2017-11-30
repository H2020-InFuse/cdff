/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DFNTestInterface.cpp
 * @date 27/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the dfn test interface class.
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
#include "DFNTestInterface.hpp"
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
DFNTestInterface::DFNTestInterface(std::string dfnName, int buttonWidth, int buttonHeight) 
	: mainInterface("Control Panel", buttonWidth, buttonHeight), parametersInterface(dfnName)
	{
	mainInterface.AddButton("Process", DFNTestInterface::ProcessCallback, this);
	}

DFNTestInterface::~DFNTestInterface()
	{

	}

void DFNTestInterface::Run()
	{
	SetupMocksAndStubs();
	SetupParameters();
	parametersInterface.CreateTrackbars();
	mainInterface.Run();
	}

void DFNTestInterface::SetDFN(DFNCommonInterface* dfn)
	{
	this->dfn = dfn;
	dfn->setConfigurationFile(filePath);
	}

void DFNTestInterface::AddParameter(std::string groupName, std::string name, int defaultValue, int maxValue)
	{
	parametersInterface.AddParameter(groupName, name, defaultValue, maxValue);
	}

void DFNTestInterface::AddParameter(std::string groupName, std::string name, double defaultValue, double maxValue, double resolution)
	{
	parametersInterface.AddParameter(groupName, name, defaultValue, maxValue, resolution);
	}

double DFNTestInterface::GetLastProcessingTimeSeconds()
	{
	return processingTime;
	}

int DFNTestInterface::GetTotalVirtualMemoryUsedKB()
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

const std::string DFNTestInterface::filePath = "../tests/ConfigurationFiles/DFNs/FeaturesExtraction2D/Conf.xml";


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void DFNTestInterface::SetupMocksAndStubs()
	{

	}

void DFNTestInterface::SetupParameters()
	{

	}

void DFNTestInterface::DisplayResult()
	{

	}

void DFNTestInterface::ProcessCallback(void* referenceToClass)
	{
	((DFNTestInterface*)referenceToClass)->ProcessCallback();
	}

void DFNTestInterface::ProcessCallback()
	{
	parametersInterface.SaveToXml(filePath);
	dfn->configure();

	clock_t begin = clock();
	dfn->process();
	clock_t end = clock();
 	processingTime = double(end - begin) / CLOCKS_PER_SEC;
	
	DisplayResult();
	}



/** @} */
