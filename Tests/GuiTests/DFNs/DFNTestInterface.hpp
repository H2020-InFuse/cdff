/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file DFNTestInterface.hpp
 * @date 27/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This is the interface for all the classes that implement a GUI test for a single DFN.
 *
 * @{
 */

#ifndef DFN_TEST_INTERFACE_HPP
#define DFN_TEST_INTERFACE_HPP


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
class DFNTestInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		DFNTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		~DFNTestInterface();

		void Run();
		void SetDFN(DFNCommonInterface* dfn);
		void AddParameter(std::string groupName, std::string name, int defaultValue, int maxValue);
		void AddParameter(std::string groupName, std::string name, double defaultValue, double maxValue, double resolution);
		void AddSignedParameter(std::string groupName, std::string name, double defaultValue, double maxValue, double resolution);

		double GetLastProcessingTimeSeconds();
		int GetTotalVirtualMemoryUsedKB();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
	protected:


	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
		MainInterface mainInterface;	
		ParametersInterface parametersInterface;
		DFNCommonInterface* dfn;
		static const std::string filePath;
		double processingTime;
	
		virtual void SetupMocksAndStubs();
		virtual void SetupParameters();
		virtual void DisplayResult();

		static void ProcessCallback(void* referenceToClass);
		void ProcessCallback();


	};

#endif

/* DFNTestInterface.hpp */
/** @} */
