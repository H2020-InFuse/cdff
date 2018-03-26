/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PerformanceTestInterface.hpp
 * @date 22/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * @brief This class is used for extensive testing of sequences of DFNs, with the aim of computing performance measures.
 *
 * PerformanceTestInterface offers basic mechanisms for executing many DFNs multiple time over many combinations of parameters and different inputs. Measure on the performance of the DFNs are stored 
 * on a file for future processing.
 *
 * A complete performance test for a sequence DFN requires the following:
 * 1) A class derived from PerformanceTestInterface, the class has to implement the SetNextInputs(), ExecuteDfns(), and ExtractMeasures() methods;
 * 2) A set of performance test configuration files, which are the same as the DFN configuration files with the exception that for every parameter it is possible to specify more than one value;
 * 3) SetNextInputs() allows you to load the inputs for a DFN;
 * 4) ExecuteDfns() allows the processing of dfns one by one, according to the tested integration logic;
 * 5) ExtractMeasures() allows you to extract measures from the output of the DFN; time and memory are already recorded by PerformanceTestInterface.
 * 
 * @{
 */

#ifndef PERFORMANCE_TEST_INTERFACE_HPP
#define PERFORMANCE_TEST_INTERFACE_HPP


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
class PerformanceTestInterface
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
		PerformanceTestInterface(std::string folderPath, std::vector<std::string> baseConfigurationFileNamesList, std::string performanceMeasuresFileName);

		/*
		* @brief The destructor does nothing.
		*
		*/
		~PerformanceTestInterface();

		/*
		* @brief The function will execute the DFN for each input provided by SetNextInputs(), and for each combination of parameters in the Configuration File defined in the constructor.
		*
		*/
		void Run();

		/*
		* @brief The function defines the main DFNs the performance test is executed upon.
		*
		*/
		void AddDfn(dfn_ci::DFNCommonInterface* dfn);

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
		void AddAggregator(std::string measure, Aggregator* aggregator, AggregationType aggregatorType);		

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:	
		typedef std::map<std::string, float> MeasuresMap;

		std::vector<dfn_ci::DFNCommonInterface*> dfnsList;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:	
		struct Parameter
			{
			unsigned dfnIndex;
			unsigned groupIndex;
			std::string name;
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

		unsigned dfnsNumber;
		static const std::string temporaryConfigurationFileNameBase;
		static const std::string temporaryConfigurationFileNameExtension;
		static const std::string aggregatorsResultsFileName;
		std::vector<std::string> baseConfigurationFilePathsList;
		std::string performanceMeasuresFilePath;
		std::string aggregatorsResultsFilePath;
		std::vector<std::string> temporaryConfigurationFilePathsList;

		std::vector<YAML::Node> configurationsList;
		std::vector<Parameter> changingParametersList;
		std::vector<AggregatorEntry> aggregatorsList;
		bool firstRun;
		bool firstMeasureTimeForCurrentInput;

		int GetTotalVirtualMemoryUsedKB();
		void SaveNewInputsLine();
		void SaveMeasures(MeasuresMap measuresMap);
		bool PrepareConfigurationFile();
		void SaveToYaml();
		void SaveRunTime(float time, unsigned numberOfTests);
		void ReadConfiguration();
		std::vector<std::string> SplitString(std::string inputString);
		void UpdateAggregators(MeasuresMap measuresMap, unsigned testNumberOnCurrentInput);
		void SaveAggregatorsResults();
		std::string ToString(AggregationType type);

		/*
		* @brief This method has to set the inputs of the DFN, it returns true if and only if an input is actually set.
		*
		* The idea is that inputs will be provided one at a time, when all inputs have already been provided the method should return false;
		*
		*/
		virtual bool SetNextInputs() = 0;

		/*
		* @brief This method has to execute the dfns integration logic.
		*
		* This method will set up the DFNs inputs, execute their process method, and retrieve their outputs. 
		*
		*/
		virtual void ExecuteDfns() = 0;
		
		/*
		* @brief This method computes a measure of the performance of the DFN as a map from Measure Name (String) to value of the measure (float).
		*
		* The idea is that the output of the DFN is accessed and evaluated. Then ad-hoc performance measures are extracted, for example by comparing the output to some expected or optimal result.
		*
		*/		
		virtual MeasuresMap ExtractMeasures() = 0;

	};

#endif

/* PerformanceTestInterface.hpp */
/** @} */
