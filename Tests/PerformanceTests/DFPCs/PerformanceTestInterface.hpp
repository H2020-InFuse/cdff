/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PerformanceTestInterface.hpp
 * @date 23/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * @brief This class is used for extensive testing of the same DFPCs, with the aim of computing performance measures.
 *
 * PerformanceTestInterface offers basic mechanisms for executing a DFPC multiple time over many combinations of parameters and different inputs. Measure on the performance of the DFPC are stored 
 * on a file for future processing.
 *
 * A complete performance test for a DFPC requires the following:
 * 1) A class derived from PerformanceTestInterface, the class has to implement the SetNextInputs() and ExtractMeasures() methods;
 * 2) A performance test configuration file, which is the same as the DFPC configuration file with the exception that for every parameter it is possible to specify more than one value;
 * 3) SetNextInputs() allows you to load the inputs for a DFPC;
 * 4) ExtractMeasures() allows you to extract measures from the output of the DFPC; time and memory are already recorded by PerformanceTestInterface.
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
#include <DFPCCommonInterface.hpp>
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
		* @param folderPath, the path containing the configuration file, the output file and a temporary DFPC configuration file will be written here as well;
		* @param baseConfigurationFileName, the name of the configuration file that contains the range of input parameters for the tests;
		* @param performanceMeasuresFileName, the name of the output file;
		*
		*/
		PerformanceTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName);

		/*
		* @brief The destructor does nothing.
		*
		*/
		~PerformanceTestInterface();

		/*
		* @brief The function defines the main DFN the performance test is executed upon.
		*
		*/
		void SetDfpc(CDFF::DFPC::DFPCCommonInterface* dfpc);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:	
		CDFF::DFPC::DFPCCommonInterface* dfpc;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:	
		void Configure();
		void Process();

		/*
		* @brief This method has to set the inputs of the DFPC, it returns true if and only if an input is actually set.
		*
		* The idea is that inputs will be provided one at a time, when all inputs have already been provided the method should return false;
		*
		*/
		virtual bool SetNextInputs() = 0;

		/*
		* @brief This method executes the DFPC once or multiple times, this is relevant as the DFPC may contain an history and multiple calls with same input yield different results.
		*
		* The idea is that a specific DFPC behaviour will be implemented here. The default behaviour is one call to the DFPC run method.
		*
		*/		
		virtual void ExecuteDfpc();
		
		/*
		* @brief This method computes a measure of the performance of the DFPC as a map from Measure Name (String) to value of the measure (float).
		*
		* The idea is that the output of the DFPC is accessed and evaluated. Then ad-hoc performance measures are extracted, for example by comparing the output to some expected or optimal result.
		*
		*/		
		virtual MeasuresMap ExtractMeasures() = 0;

	};

#endif

/* PerformanceTestInterface.hpp */
/** @} */
