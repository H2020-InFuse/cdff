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
	: buttonList(windowName, buttonWidth, buttonHeight)
	{
	shutdownNow = false;
	}



void MainInterface::Run()
	{
	buttonList.AddButton("Exit", std::bind(&MainInterface::ExitCallback, this));
	buttonList.Display();
	while(!shutdownNow)
		{
		cv::waitKey(refreshRate);
		}
	}

void MainInterface::AddButton(std::string option, ButtonsInterface::on_button_clicked_cb_t callback)
	{
		buttonList.AddButton(option, callback);
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

void MainInterface::ExitCallback()
	{
	shutdownNow = true;
	}



/** @} */
