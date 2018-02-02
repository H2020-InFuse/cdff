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

using namespace dfn_ci;

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
		void AddDFN(DFNCommonInterface* dfn, std::string dfnName);
		void AddParameter(DFNCommonInterface* dfn, std::string groupName, std::string name, int defaultValue, int maxValue);
		void AddParameter(DFNCommonInterface* dfn, std::string groupName, std::string name, double defaultValue, double maxValue, double resolution);

		double GetTotalProcessingTimeSeconds();
		double GetLastProcessingTimeSeconds(DFNCommonInterface* dfn);
		int GetTotalVirtualMemoryUsedKB();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:
		unsigned GetDfnIndex(DFNCommonInterface* dfn);

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		MainInterface mainInterface;	
		std::vector<ParametersInterface> parametersInterfaceList;

		std::vector<std::string> dfnsNamesList;
		std::vector<DFNCommonInterface*> dfnsList;
		static const std::string baseFilePath;
		std::vector<double> processingTime;
	
		virtual void SetupMocksAndStubs();
		virtual void SetupParameters();
		virtual void DisplayResult();

		void CleanProcessingTime();
		std::string GetDfnFilePath(unsigned dfnIndex);
		void ConfigureDFNs();
		static void ProcessCallback(void* referenceToClass);
		void ProcessCallback();

		virtual void ResetProcess() = 0;
		virtual bool IsProcessCompleted() = 0;
		virtual unsigned PrepareNextDfn() = 0;

	};

#endif

/* DFNsIntegrationTestInterface.hpp */
/** @} */
