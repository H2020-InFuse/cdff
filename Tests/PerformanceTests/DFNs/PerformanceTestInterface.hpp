/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PerformanceTestInterface.hpp
 * @date 16/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class is used for extensive testing of the same DFN, with the aim of computing performance measures.
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
		PerformanceTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName);
		~PerformanceTestInterface();
		void Run();
		void SetDfn(dfn_ci::DFNCommonInterface* dfn);

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:	
		typedef std::map<std::string, float> MeasuresMap;

		dfn_ci::DFNCommonInterface* dfn;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:	
		struct Parameter
			{
			unsigned groupIndex;
			std::string name;
			unsigned optionsNumber;
			unsigned currentOption;
			std::vector<std::string> optionsList;
			};

		static const std::string temporaryConfigurationFileName;
		std::string baseConfigurationFilePath;
		std::string performanceMeasuresFilePath;
		std::string temporaryConfigurationFilePath;

		YAML::Node configuration;
		std::vector<Parameter> changingParametersList;
		bool firstRun;
		bool firstMeasureTimeForCurrentInput;

		int GetTotalVirtualMemoryUsedKB();
		void SaveNewInputsLine();
		virtual bool SetNextInputs() = 0;
		virtual MeasuresMap ExtractMeasures() = 0;
		void SaveMeasures(MeasuresMap measuresMap);
		bool PrepareConfigurationFile();
		void SaveToYaml();
		void SaveRunTime(float time, unsigned numberOfTests);
		void ReadConfiguration();
		std::vector<std::string> SplitString(std::string inputString);

	};

#endif

/* PerformanceTestInterface.hpp */
/** @} */
