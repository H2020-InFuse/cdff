/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PerformanceTestBase.cpp
 * @date 27/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the performance test base class.
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
#include "PerformanceTestBase.hpp"
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
PerformanceTestBase::PerformanceTestBase(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName)
	{
	configurationFilesNumber = baseConfigurationFileNamesList.size();
	for(unsigned configurationFileIndex = 0; configurationFileIndex < configurationFilesNumber; configurationFileIndex++)
		{
		std::stringstream baseConfigurationFileStream, temporaryConfigurationFileStream;	
		baseConfigurationFileStream << folderPath << "/" << baseConfigurationFileNamesList.at(configurationFileIndex);
		temporaryConfigurationFileStream << folderPath << "/" << temporaryConfigurationFileNameBase << configurationFileIndex << temporaryConfigurationFileNameExtension;

		baseConfigurationFilePathsList.push_back(baseConfigurationFileStream.str());
		temporaryConfigurationFilePathsList.push_back(temporaryConfigurationFileStream.str());
		}

	std::stringstream performanceMeasuresFileStream, aggregatorsResultsFileStream;	
	performanceMeasuresFileStream << folderPath << "/" << performanceMeasuresFileName;
	aggregatorsResultsFileStream << folderPath << "/" << aggregatorsResultsFileName;
	performanceMeasuresFilePath = performanceMeasuresFileStream.str();
	aggregatorsResultsFilePath = aggregatorsResultsFileStream.str();

	ReadPerformanceConfigurationFiles();
	}

PerformanceTestBase::~PerformanceTestBase()
	{

	}

void PerformanceTestBase::Run()
	{
	while ( SetNextInputs() )
		{
		unsigned numberOfTests = 0;
		clock_t beginRun = clock();
		SaveNewInputsLine();
		firstRunOnInput = true;

		while ( PrepareTemporaryConfigurationFiles() )
			{
			numberOfTests++;
			Configure();

			clock_t begin = clock();
			Process();
			clock_t end = clock();

			MeasuresMap measuresMap = ExtractMeasures();
			measuresMap["ProcessingTimeS"] = float(end - begin) / CLOCKS_PER_SEC;
			measuresMap["VirtualMemoryKB"] = GetTotalVirtualMemoryUsedKB();

			SaveMeasures(measuresMap);
			UpdateAggregators(measuresMap, numberOfTests);
			}

		clock_t endRun = clock();
		SaveRunTime( float(endRun - beginRun) / CLOCKS_PER_SEC, numberOfTests );
		numberOfTests = 0;
		}

	SaveAggregatorsResults();
	}

void PerformanceTestBase::AddAggregator(std::string measure, Aggregator* aggregator, AggregationType aggregatorType)
	{
	AggregatorEntry entry;
	entry.measure = measure;
	entry.aggregator = aggregator;
	entry.aggregatorType = aggregatorType;

	aggregatorsList.push_back(entry);
	}

/* --------------------------------------------------------------------------
 *
 * Protected Member Functions
 *
 * --------------------------------------------------------------------------
 */




/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const std::string PerformanceTestBase::temporaryConfigurationFileNameBase = "PerformanceTest_TemporaryDFN";
const std::string PerformanceTestBase::temporaryConfigurationFileNameExtension = ".yaml";
const std::string PerformanceTestBase::aggregatorsResultsFileName = "AggregatedMeasures.txt";


/* --------------------------------------------------------------------------
 *
 * Private Member Functions 
 * (these functions deals with the reading of the range of parameters values, and creation of
 * temporary configuration files for each combination of parameters)
 *
 * --------------------------------------------------------------------------
 */
void PerformanceTestBase::ReadPerformanceConfigurationFiles()
	{
	for(unsigned configurationFileIndex = 0; configurationFileIndex < configurationFilesNumber; configurationFileIndex++)
		{
		configurationsList.push_back( YAML::LoadFile( baseConfigurationFilePathsList.at(configurationFileIndex) ) );
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
					parameter.configurationFileIndex = configurationFileIndex;
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

std::vector<std::string> PerformanceTestBase::SplitString(std::string inputString)
	{
	std::vector<std::string> componentsList;
	boost::split(componentsList, inputString, boost::is_any_of(", "), boost::token_compress_on);
	return componentsList;
	}

bool PerformanceTestBase::PrepareTemporaryConfigurationFiles()	
	{
	if (firstRunOnInput)
		{
		firstRunOnInput = false;
		SaveTemporaryConfigurationFiles();
		return true;
		}

	for(int parameterIndex = changingParametersList.size()-1; parameterIndex >= 0; parameterIndex--)
		{
		Parameter& parameter = changingParametersList.at(parameterIndex);

		parameter.currentOption = (parameter.currentOption + 1) % parameter.optionsNumber;
		YAML::Node& configuration = configurationsList.at(parameter.configurationFileIndex);
		YAML::Node group = configuration[parameter.groupIndex];
		group[parameter.name] = parameter.optionsList.at(parameter.currentOption);

		if (parameter.currentOption > 0)
			{
			SaveTemporaryConfigurationFiles();
			return true;
			}
		}

	return false;
	}

void PerformanceTestBase::SaveTemporaryConfigurationFiles()
	{
	for(unsigned configurationFileIndex = 0; configurationFileIndex < configurationFilesNumber; configurationFileIndex++)
		{
		std::ofstream yamlFile(temporaryConfigurationFilePathsList.at(configurationFileIndex).c_str());
		yamlFile << configurationsList.at(configurationFileIndex);
		yamlFile.close();
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions 
 * (these functions deals with accessing the output file for the record of performance measures)
 *
 * --------------------------------------------------------------------------
 */

void PerformanceTestBase::SaveNewInputsLine()
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

void PerformanceTestBase::SaveMeasures(MeasuresMap measuresMap)
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


void PerformanceTestBase::SaveRunTime(float time, unsigned numberOfTests)	
	{
	std::ofstream measuresFile(performanceMeasuresFilePath.c_str(), std::ios::app);

	measuresFile << "Total Run Time for input (including instrumentation): " << time << "\n";
	measuresFile << "Average time for input (including instrumentation): " << time / static_cast<float>(numberOfTests) << "\n";
	measuresFile.close();
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions 
 * (This function deals with the measurement of total memory used on unix systems during the testing)
 *
 * --------------------------------------------------------------------------
 */
int PerformanceTestBase::GetTotalVirtualMemoryUsedKB()
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
 * Private Member Functions 
 * (This function deals with adding data to the aggregators and saving the result of the aggregation)
 *
 * --------------------------------------------------------------------------
 */
void PerformanceTestBase::UpdateAggregators(MeasuresMap measuresMap, unsigned testNumberOnCurrentInput)
	{
	for(unsigned aggregatorIndex = 0; aggregatorIndex < aggregatorsList.size(); aggregatorIndex++)
		{
		AggregatorEntry entry =  aggregatorsList.at(aggregatorIndex);
		
		if( measuresMap.find( entry.measure ) == measuresMap.end() )
			{
			continue;
			}
		double value = measuresMap[ entry.measure ];

		switch(entry.aggregatorType)
			{
			case VARIABLE_PARAMETERS_FIXED_INPUTS:
				{
				unsigned numberOfAggregatorChannels = entry.aggregator->GetNumberOfChannels();
				if (testNumberOnCurrentInput == 1)
					{
					entry.aggregator->AddMeasure( value, numberOfAggregatorChannels);
					}
				else
					{
					entry.aggregator->AddMeasure( value, numberOfAggregatorChannels-1);
					}
				break;
				}
			case FIXED_PARAMETERS_VARIABLE_INPUTS:
				{
				entry.aggregator->AddMeasure( value, testNumberOnCurrentInput-1);
				break;
				}
			case VARIABLE_PARAMETERS_VARIABLE_INPUTS:
				{
				entry.aggregator->AddMeasure( value );
				break;
				}
			default:
				{
				ASSERT(false, "Unhandler aggregator type in PerformanceTestBase::UpdateAggregators");
				}
			}
		}
	}

void PerformanceTestBase::SaveAggregatorsResults()
	{
	std::ofstream aggregatorsFile(aggregatorsResultsFilePath.c_str());	

	for(unsigned aggregatorIndex = 0; aggregatorIndex < aggregatorsList.size(); aggregatorIndex++)
		{
		AggregatorEntry entry =  aggregatorsList.at(aggregatorIndex);
		std::vector<double> result = entry.aggregator->Aggregate();
		aggregatorsFile << entry.measure << " " << AggregationTypeToString(entry.aggregatorType) << "\n";
		for(unsigned index = 0; index < result.size(); index++)
			{
			SaveParametersAndValueInAggregatorFile(aggregatorsFile, index, result.at(index));
			}
		aggregatorsFile << "\n";
		}

	aggregatorsFile.close();
	}

std::string PerformanceTestBase::AggregationTypeToString(AggregationType type)
	{
	if (type == FIXED_PARAMETERS_VARIABLE_INPUTS)
		{
		return "fixed parameters variable inputs";
		}
	else if (type == VARIABLE_PARAMETERS_FIXED_INPUTS)
		{
		return "variable parameters fixed inputs";
		}
	else if (type == VARIABLE_PARAMETERS_VARIABLE_INPUTS)
		{
		return "variable parameters variable inputs";
		}
	else
		{
		ASSERT(false, "Unhandler aggregator type in PerformanceTestBase::AggregationTypeToString");
		}

	return "";
	}

void PerformanceTestBase::SaveParametersAndValueInAggregatorFile(std::ofstream& file, unsigned index, float value)
	{
	/* Print an header only before the first row */
	if (index == 0)
		{
		for(std::vector<Parameter>::iterator parameter = changingParametersList.begin(); parameter != changingParametersList.end(); parameter++)
			{
			file << parameter->name << " ";
			}
		file << "AggregatorResult" << "\n";
		}

	unsigned residualIndex = index;
	for(std::vector<Parameter>::iterator parameter = changingParametersList.begin(); parameter != changingParametersList.end(); parameter++)
		{
		unsigned optionIndex = residualIndex % parameter->optionsNumber;
		file << parameter->optionsList.at(optionIndex) << " ";
		residualIndex = residualIndex / parameter->optionsNumber;
		}
	file << value << "\n";	
	}

/** @} */
