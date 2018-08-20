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

/**
 * The DFNTestInterface is the base class for all the GUI Tests.
 */
class DFNTestInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		DFNTestInterface(std::string dfnName, int buttonWidth, int buttonHeight);
		virtual ~DFNTestInterface() = default;

		void Run();
		void SetDFN(CDFF::DFN::DFNCommonInterface* dfn);
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
		CDFF::DFN::DFNCommonInterface* dfn;
		static const std::string filePath;
		double processingTime;
	
		virtual void SetupMocksAndStubs();
		virtual void SetupParameters();
		virtual void DisplayResult();

		void ProcessCallback();


	};

#endif

/* DFNTestInterface.hpp */
/** @} */
