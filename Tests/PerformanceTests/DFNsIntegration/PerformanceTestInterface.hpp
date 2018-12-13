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
#include <DFNCommonInterface.hpp>
#include <PerformanceTests/PerformanceTestBase.hpp>


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class PerformanceTestInterface : public PerformanceTestBase
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
		PerformanceTestInterface(const std::string& folderPath, const std::vector<std::string>& baseConfigurationFileNamesList, const std::string& performanceMeasuresFileName);

		/*
		* @brief The destructor does nothing.
		*
		*/
		~PerformanceTestInterface();

		/*
		* @brief The function defines the main DFNs the performance test is executed upon.
		*
		*/
		void AddDfn(CDFF::DFN::DFNCommonInterface* dfn);		

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:
		std::vector<CDFF::DFN::DFNCommonInterface*> dfnsList;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:	
		void Configure() override;
		void Process() override;

		/*
		* @brief This method has to set the inputs of the DFN, it returns true if and only if an input is actually set.
		*
		* The idea is that inputs will be provided one at a time, when all inputs have already been provided the method should return false;
		*
		*/
		virtual bool SetNextInputs() override = 0;

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
