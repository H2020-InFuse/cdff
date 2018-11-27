/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DFNsIntegrationTestInterface.hpp
 * @date 02/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This is the interface for all the classes that implement a GUI test for multiple DFNs.
 *
 * @{
 */

#ifndef DFNS_INTEGRATION_TEST_INTERFACE_HPP
#define DFNS_INTEGRATION_TEST_INTERFACE_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <GuiTests/MainInterface.hpp>
#include <GuiTests/ParametersInterface.hpp>
#include <DFNCommonInterface.hpp>
#include <stdlib.h>
#include <string>
#include <vector>

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class DFNsIntegrationTestInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		DFNsIntegrationTestInterface(int buttonWidth, int buttonHeight);
		~DFNsIntegrationTestInterface();

		void Run();
		void AddDFN(CDFF::DFN::DFNCommonInterface* dfn, std::string dfnName);
		void AddParameter(CDFF::DFN::DFNCommonInterface* dfn, std::string groupName, std::string name, int defaultValue, int maxValue);
		void AddParameter(CDFF::DFN::DFNCommonInterface* dfn, std::string groupName, std::string name, double defaultValue, double maxValue, double resolution);
		void AddSignedParameter(CDFF::DFN::DFNCommonInterface* dfn, std::string groupName, std::string name, double defaultValue, double maxValue, double resolution);

		double GetTotalProcessingTimeSeconds();
		double GetLastProcessingTimeSeconds(unsigned timeIndex);
		int GetTotalVirtualMemoryUsedKB();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:
		unsigned GetDfnIndex(CDFF::DFN::DFNCommonInterface* dfn);

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		MainInterface mainInterface;	
		std::vector<ParametersInterface> parametersInterfaceList;

		std::vector<std::string> dfnsNamesList;
		std::vector<CDFF::DFN::DFNCommonInterface*> dfnsList;
		static const std::string baseFilePath;
		std::vector<double> processingTime;
		double totalProcessingTime;
	
		virtual void SetupMocksAndStubs();
		virtual void SetupParameters();
		virtual void DisplayResult();

		void CleanProcessingTime();
		std::string GetDfnFilePath(unsigned dfnIndex);
		void ConfigureDFNs();
		void ProcessCallback();

		virtual void ResetProcess() = 0;
		virtual bool IsProcessCompleted() = 0;
		virtual void UpdateState() = 0;
		virtual CDFF::DFN::DFNCommonInterface* PrepareNextDfn() = 0;


	};

#endif

/* DFNsIntegrationTestInterface.hpp */
/** @} */
