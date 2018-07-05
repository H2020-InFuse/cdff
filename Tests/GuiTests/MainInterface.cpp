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
	ButtonsInterface::ButtonStyle exit_style;
	exit_style.backgroundColor = cv::Scalar(60, 76, 231); // Red - Alizarin
	exit_style.textColor = cv::Scalar(255, 255, 255);    // White

	buttonList.AddButton("Exit", std::bind(&MainInterface::ExitCallback, this), exit_style);
	buttonList.Display();
	while(!shutdownNow)
		{
		cv::waitKey(refreshRate);
		}
	}

void MainInterface::AddButton(std::string option, ButtonsInterface::ButtonClickedCallback callback)
	{
		buttonList.AddButton(option, callback);
	}

void MainInterface::AddButton(
	std::string option,
	ButtonsInterface::ButtonClickedCallback callback,
	ButtonsInterface::ButtonStyle style)
	{
	buttonList.AddButton(option, callback, style);
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
