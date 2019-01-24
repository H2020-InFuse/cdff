/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PerformanceTestBase.hpp
 * @date 27/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * @brief This class is used as a base class for the performance test class, it offers common basic mechanisms for executing a performance test.
 *
 * PerformanceTestBase offers basic mechanisms for executing either a DFN, an integration of DFNs or a DFPCs multiple time over many combinations of parameters and different inputs. 
 * Measure on the performance of the DFNs are stored on a file for future processing.
 *
 * This class is specialized in a class for execution of a DFN, an integration of DFNs or a DFPC.
 * 
 * @{
 */

#ifndef PERFORMANCE_TEST_BASE_HPP
#define PERFORMANCE_TEST_BASE_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <stdlib.h>
#include <string>
#include <vector>
#include <DFNCommonInterface.hpp>
#include <map>
#include <yaml-cpp/yaml.h>
#include "Aggregator.hpp"


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PerformanceTestBase
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		/*
		* @brief The constructor requires the folder were the configuration files and outputs will be stored, it also requires the configuration file name and the output file name
		*
		* @param folderPath, the path containing the configuration file, the output file and a temporary DFN configuration file will be written here as well;
		* @param baseConfigurationFileNamesList, the name of the configuration files that contains the range of input parameters for the tests;
		* @param performanceMeasuresFileName, the name of the output file;
		*
		*/
		PerformanceTestBase(const std::string& folderPath, const std::vector<std::string>& baseConfigurationFileNamesList, const std::string& performanceMeasuresFileName);

		/*
		* @brief The destructor does nothing.
		*
		*/
		~PerformanceTestBase();

		/*
		* @brief The function will execute a Configure and Process methods for each input provided by SetNextInputs(), and for each combination of parameters in the Configuration File 
		* defined in the constructor. Process and Configure will be defined in the specialized class that deals with DFNs, integrations of DFNs or DFPCs.
		*
		*/
		void Run();

		/*
		* @brief defines the way an aggregator is used. See next menthod
		*/
		enum AggregationType
			{
			FIXED_PARAMETERS_VARIABLE_INPUTS,
			VARIABLE_PARAMETERS_FIXED_INPUTS,
			VARIABLE_PARAMETERS_VARIABLE_INPUTS
			};

		/*
		* @brief the function defines an aggregator of measures.
		*
		* @param measure, this is the string identifier of the test measure the aggregator will work upon;
		* @param aggregator, this is the aggretor object that will perform the aggregation;
		* @param aggregatorType, this defines whether all measures are aggregated (VARIABLE_PARAMETERS_VARIABLE_INPUTS), whether the measure with the same inputs are aggregated
		*	(VARIABLE_PARAMETERS_FIXED_INPUTS), or whether the mesaures with the same parameters are aggregated (FIXED_PARAMETERS_VARIABLE_INPUTS);
		*
		*/
		void AddAggregator(const std::string& measure, Aggregator* aggregator, AggregationType aggregatorType);		

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:	
		typedef std::map<std::string, float> MeasuresMap;

		std::vector<std::string> temporaryConfigurationFilePathsList;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		struct Parameter
			{
			unsigned configurationFileIndex;
			std::string baseLine;
			unsigned optionsNumber;
			unsigned currentOption;
			std::vector<std::string> optionsList;
			};

		struct AggregatorEntry
			{
			std::string measure;
			Aggregator* aggregator;
			AggregationType aggregatorType;
			};

		unsigned configurationFilesNumber;
		std::vector<YAML::Node> configurationsList;
		std::vector<Parameter> changingParametersList;
		std::vector<AggregatorEntry> aggregatorsList;

		bool firstRunOnInput;
		bool firstMeasureTimeForCurrentInput;
	
		static const std::string temporaryConfigurationFileNameBase;
		static const std::string temporaryConfigurationFileNameExtension;
		static const std::string aggregatorsResultsFileName;
		std::vector<std::string> baseConfigurationFilePathsList;
		std::string performanceMeasuresFilePath;
		std::string aggregatorsResultsFilePath;

		void ReadPerformanceConfigurationFiles();
		std::vector<std::string> SplitString(std::string inputString);
		bool PrepareTemporaryConfigurationFiles();
		void SaveTemporaryConfigurationFiles();
		std::string RemoveStartAndEndSpaces(const std::string& word);

		void SaveNewInputsLine();
		void SaveMeasures(MeasuresMap measuresMap);
		void SaveRunTime(float time, unsigned numberOfTests);

		int GetTotalVirtualMemoryUsedKB();

		void UpdateAggregators(MeasuresMap measuresMap, unsigned testNumberOnCurrentInput);
		void SaveAggregatorsResults();
		std::string AggregationTypeToString(AggregationType type);
		void SaveParametersAndValueInAggregatorFile(std::ofstream& file, unsigned index, float value);

		/*
		* @brief This method performs the necessary configuration required to execute a test on a given input for a give set of parameters
		*
		*/
		virtual void Configure() = 0;

		/*
		* @brief This method performs executes a test on a given input for a give set of parameters
		*
		*/
		virtual void Process() = 0;

		/*
		* @brief This method has to set the inputs of the test, it returns true if and only if an input is actually set.
		*
		* The idea is that inputs will be provided one at a time, when all inputs have already been provided the method should return false;
		*
		*/
		virtual bool SetNextInputs() = 0;
		
		/*
		* @brief This method computes a measure of the performance of the test as a map from Measure Name (String) to value of the measure (float).
		*
		* The idea is that the output of the test is accessed and evaluated. Then ad-hoc performance measures are extracted, for example by comparing the output to some expected or optimal result.
		*
		*/		
		virtual MeasuresMap ExtractMeasures() = 0;

	};

#endif

/* PerformanceTestBase.hpp */
/** @} */
