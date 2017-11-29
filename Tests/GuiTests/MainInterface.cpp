/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MainInterface.cpp
 * @date 27/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the main interface class.
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
#include "MainInterface.hpp"


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
MainInterface::MainInterface(std::string windowName, int buttonWidth, int buttonHeight) 
	: innerInterface(windowName, buttonWidth, buttonHeight)
	{
	shutdownNow = false;
	
	}

MainInterface::~MainInterface()
	{

	}

void MainInterface::Run()
	{
	innerInterface.AddButton("Exit", MainInterface::ExitCallback, this);
	innerInterface.Display();
	while(!shutdownNow)
		{
		cv::waitKey(refreshRate);
		}
	}

void MainInterface::AddButton(std::string option, void (*callback)(void*), void* userdata)
	{
	innerInterface.AddButton(option, callback, userdata);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const unsigned MainInterface::refreshRate = 200;



/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void MainInterface::ExitCallback(void* instance)
	{
	((MainInterface*)instance)->ExitCallback();
	}

void MainInterface::ExitCallback()
	{
	shutdownNow = true;
	}



/** @} */
