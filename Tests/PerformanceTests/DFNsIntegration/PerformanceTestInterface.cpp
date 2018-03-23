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
PerformanceTestInterface::PerformanceTestInterface(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName)
	{
	dfnsNumber = baseConfigurationFileNamesList.size();
	for(unsigned dfnIndex = 0; dfnIndex < dfnsNumber; dfnIndex++)
		{
		std::stringstream baseConfigurationFileStream, temporaryConfigurationFileStream;	
		baseConfigurationFileStream << folderPath << "/" << baseConfigurationFileNamesList.at(dfnIndex);
		temporaryConfigurationFileStream << folderPath << "/" << temporaryConfigurationFileNameBase << dfnIndex << temporaryConfigurationFileNameExtension;

		baseConfigurationFilePathsList.push_back(baseConfigurationFileStream.str());
		temporaryConfigurationFilePathsList.push_back(temporaryConfigurationFileStream.str());
		}

	std::stringstream performanceMeasuresFileStream;	
	performanceMeasuresFileStream << folderPath << "/" << performanceMeasuresFileName;
	performanceMeasuresFilePath = performanceMeasuresFileStream.str();

	ReadConfiguration();
	firstRun = true;
	}

PerformanceTestInterface::~PerformanceTestInterface()
	{

	}

void PerformanceTestInterface::Run()
	{
	ASSERT( dfnsNumber == dfnsList.size(), "The number of dfns does not match the number of configuration files");
	while ( SetNextInputs() )
		{
		unsigned numberOfTests = 0;
		clock_t beginRun = clock();
		SaveNewInputsLine();

		while ( PrepareConfigurationFile() )
			{
			numberOfTests++;
			for(unsigned dfnIndex = 0; dfnIndex < dfnsNumber; dfnIndex++)
				{
				dfn_ci::DFNCommonInterface* dfn = dfnsList.at(dfnIndex);
				dfn->setConfigurationFile( temporaryConfigurationFilePathsList.at(dfnIndex) );
				dfn->configure();
				}

			clock_t begin = clock();
			ExecuteDfns();
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

void PerformanceTestInterface::AddDfn(dfn_ci::DFNCommonInterface* dfn)
	{
	dfnsList.push_back(dfn);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const std::string PerformanceTestInterface::temporaryConfigurationFileNameBase = "PerformanceTest_TemporaryDFN";
const std::string PerformanceTestInterface::temporaryConfigurationFileNameExtension = ".yaml";


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
		}
	else
		{
		measuresFile.open(performanceMeasuresFilePath.c_str(), std::ios::app);
		firstTime = false;
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
	if (firstRun)
		{
		firstRun = false;
		SaveToYaml();
		return true;
		}

	for(int parameterIndex = changingParametersList.size()-1; parameterIndex >= 0; parameterIndex--)
		{
		Parameter& parameter = changingParametersList.at(parameterIndex);

		parameter.currentOption = (parameter.currentOption + 1) % parameter.optionsNumber;
		YAML::Node& configuration = configurationsList.at(parameter.dfnIndex);
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
	for(unsigned dfnIndex = 0; dfnIndex < dfnsNumber; dfnIndex++)
		{
		std::ofstream yamlFile(temporaryConfigurationFilePathsList.at(dfnIndex).c_str());
		yamlFile << configurationsList.at(dfnIndex);
		yamlFile.close();
		}
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
	for(unsigned dfnIndex = 0; dfnIndex < dfnsNumber; dfnIndex++)
		{
		configurationsList.push_back( YAML::LoadFile( baseConfigurationFilePathsList.at(dfnIndex) ) );
		YAML::Node& configuration = configurationsList.at( configurationsList.size() - 1 );

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
	}

std::vector<std::string> PerformanceTestInterface::SplitString(std::string inputString)
	{
	std::vector<std::string> componentsList;
	boost::split(componentsList, inputString, boost::is_any_of(", "), boost::token_compress_on);
	return componentsList;
	}

/** @} */
