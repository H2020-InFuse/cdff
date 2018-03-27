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
#include <ctime>
#include <stdio.h>
#include <string.h>
#include <fstream>
#include <stdlib.h>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>
#include <Errors/Assert.hpp>


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PerformanceTestInterface::PerformanceTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName)
	{
	std::stringstream baseConfigurationFileStream, performanceMeasuresFileStream, temporaryConfigurationFileStream;	
	baseConfigurationFileStream << folderPath << "/" << baseConfigurationFileName;
	performanceMeasuresFileStream << folderPath << "/" << performanceMeasuresFileName;
	temporaryConfigurationFileStream << folderPath << "/" << temporaryConfigurationFileName;

	baseConfigurationFilePath = baseConfigurationFileStream.str();
	performanceMeasuresFilePath = performanceMeasuresFileStream.str();
	temporaryConfigurationFilePath = temporaryConfigurationFileStream.str();

	ReadConfiguration();
	firstRun = true;
	}

PerformanceTestInterface::~PerformanceTestInterface()
	{

	}

void PerformanceTestInterface::Run()
	{
	while ( SetNextInputs() )
		{
		unsigned numberOfTests = 0;
		clock_t beginRun = clock();
		SaveNewInputsLine();
		firstRunOnInput = true;

		while ( PrepareConfigurationFile() )
			{
			numberOfTests++;
			dfn->setConfigurationFile(temporaryConfigurationFilePath);
			dfn->configure();

			clock_t begin = clock();
			dfn->process();
			clock_t end = clock();

			MeasuresMap measuresMap = ExtractMeasures();
			measuresMap["ProcessingTimeS"] = float(end - begin) / CLOCKS_PER_SEC;
			measuresMap["VirtualMemoryKB"] = GetTotalVirtualMemoryUsedKB();

			SaveMeasures(measuresMap);
			}

		clock_t endRun = clock();
		SaveRunTime( float(endRun - beginRun) / CLOCKS_PER_SEC, numberOfTests );
		numberOfTests = 0;
		}


	firstRun = true;
	}

void PerformanceTestInterface::SetDfn(dfn_ci::DFNCommonInterface* dfn)
	{
	this->dfn = dfn;
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const std::string PerformanceTestInterface::temporaryConfigurationFileName = "PerformanceTest_TemporaryDFN.yaml";


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
int PerformanceTestInterface::GetTotalVirtualMemoryUsedKB()
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

void PerformanceTestInterface::SaveNewInputsLine()
	{
	static bool firstTime = true;
	static unsigned inputNumber = 0;
	std::ofstream measuresFile;

	if (firstTime)
		{
		measuresFile.open(performanceMeasuresFilePath.c_str());
		firstTime = false;
		}
	else
		{
		measuresFile.open(performanceMeasuresFilePath.c_str(), std::ios::app);
		}

	inputNumber++;
	measuresFile << "Input N. " << inputNumber << "\n";
	firstMeasureTimeForCurrentInput = true;
	measuresFile.close();
	}

void PerformanceTestInterface::SaveMeasures(MeasuresMap measuresMap)
	{
	static unsigned testIdentifier = 0;
	testIdentifier++;

	std::ofstream measuresFile(performanceMeasuresFilePath.c_str(), std::ios::app);

	if (firstMeasureTimeForCurrentInput)
		{	
		measuresFile << "Identifier ";
		for(unsigned parameterIndex = 0; parameterIndex < changingParametersList.size(); parameterIndex++)
			{
			measuresFile << changingParametersList.at(parameterIndex).name << " ";
			}
		for(MeasuresMap::iterator iterator = measuresMap.begin(); iterator != measuresMap.end(); iterator++)
			{
			measuresFile << iterator->first <<" ";
			}
		measuresFile << "\n";
		firstMeasureTimeForCurrentInput = false;
		}
	
	measuresFile << testIdentifier << " ";
	for(unsigned parameterIndex = 0; parameterIndex < changingParametersList.size(); parameterIndex++)
		{
		unsigned currentOption = changingParametersList.at(parameterIndex).currentOption;
		measuresFile << changingParametersList.at(parameterIndex).optionsList.at(currentOption) << " ";
		}
	for(MeasuresMap::iterator iterator = measuresMap.begin(); iterator != measuresMap.end(); iterator++)
		{
		measuresFile << iterator->second <<" ";
		}
	measuresFile << "\n";	
	measuresFile.close();
	}

bool PerformanceTestInterface::PrepareConfigurationFile()	
	{
	if (firstRun || firstRunOnInput)
		{
		firstRun = false;
		firstRunOnInput = false;
		SaveToYaml();
		return true;
		}

	for(int parameterIndex = changingParametersList.size()-1; parameterIndex >= 0; parameterIndex--)
		{
		Parameter& parameter = changingParametersList.at(parameterIndex);

		parameter.currentOption = (parameter.currentOption + 1) % parameter.optionsNumber;
		YAML::Node group = configuration[parameter.groupIndex];
		group[parameter.name] = parameter.optionsList.at(parameter.currentOption);

		if (parameter.currentOption > 0)
			{
			SaveToYaml();
			return true;
			}
		}

	return false;
	}

void PerformanceTestInterface::SaveToYaml()
	{
	std::ofstream yamlFile(temporaryConfigurationFilePath.c_str());
	yamlFile << configuration;
	yamlFile.close();
	}

void PerformanceTestInterface::SaveRunTime(float time, unsigned numberOfTests)	
	{
	std::ofstream measuresFile(performanceMeasuresFilePath.c_str(), std::ios::app);

	measuresFile << "Total Run Time for input (including instrumentation): " << time << "\n";
	measuresFile << "Average time for input (including instrumentation): " << time / static_cast<float>(numberOfTests) << "\n";
	measuresFile.close();
	}

void PerformanceTestInterface::ReadConfiguration()
	{
	configuration= YAML::LoadFile( baseConfigurationFilePath );
	for(unsigned groupIndex = 0; groupIndex < configuration.size(); groupIndex++)
		{
		YAML::Node group = configuration[groupIndex];
		for(YAML::const_iterator iterator = group.begin(); iterator != group.end(); iterator++)
			{
			std::string parameterName = iterator->first.as<std::string>();
			std::vector<std::string> parameterValuesList = SplitString( iterator->second.as<std::string>() );
			if (parameterValuesList.size() > 1)
				{
				Parameter parameter;
				parameter.groupIndex = groupIndex;
				parameter.name = parameterName;
				parameter.optionsNumber = parameterValuesList.size();
				parameter.currentOption = 0;
				parameter.optionsList = parameterValuesList;
				group[parameterName] = parameterValuesList.at(0);
				changingParametersList.push_back(parameter);
				}
			}
		}
	}

std::vector<std::string> PerformanceTestInterface::SplitString(std::string inputString)
	{
	std::vector<std::string> componentsList;
	boost::split(componentsList, inputString, boost::is_any_of(", "), boost::token_compress_on);
	return componentsList;
	}

/** @} */
