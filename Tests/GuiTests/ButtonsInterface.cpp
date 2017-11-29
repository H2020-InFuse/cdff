/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ButtonsInterface.cpp
 * @date 24/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the buttons interface class.
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
#include "ButtonsInterface.hpp"


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
ButtonsInterface::ButtonsInterface(std::string windowName, int buttonWidth, int buttonHeight) 
	: windowName(windowName)
	{
	cv::namedWindow(windowName, CV_WINDOW_NORMAL);
	this->buttonWidth = buttonWidth;	
	this->buttonHeight = buttonHeight;
	cv::setMouseCallback( windowName, ButtonsInterface::OnMouseCallback, this);
	}

ButtonsInterface::~ButtonsInterface()
	{

	}

void ButtonsInterface::Display()
	{
	int imageColumns = buttonWidth+2*buttonPadding;
	int imageRows = buttonPadding + (buttonHeight + buttonPadding)*buttonsList.size();
	buttonsImage = cv::Mat(imageRows, imageColumns, CV_8UC3, backgroundColor);
	for(std::vector<Button>::iterator button = buttonsList.begin(); button != buttonsList.end(); button++)
		{
		cv::rectangle(buttonsImage, button->topLeftCorner, button->bottomRightCorner, buttonColor, -1, 8);
		cv::Point TextStartPoint(button->topLeftCorner.x + xTextPadding, button->bottomRightCorner.y -yTextPadding);
		cv::putText(buttonsImage, button->option, TextStartPoint, cv::FONT_HERSHEY_SIMPLEX, 0.6, textColor, 2, 8);		
		}
	cv::imshow(windowName, buttonsImage);
	}

void ButtonsInterface::AddButton(std::string option, void (*callback)(void*), void* userdata)
	{
	Button button;
	button.topLeftCorner.x = buttonPadding;
	button.topLeftCorner.y = buttonPadding + (buttonHeight+buttonPadding) * buttonsList.size();
	button.bottomRightCorner.x = button.topLeftCorner.x + buttonWidth; 
	button.bottomRightCorner.y = button.topLeftCorner.y + buttonHeight;
	button.option = option;
	button.callback = callback;
	button.userdataForCallback = userdata;
	buttonsList.push_back(button);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const int ButtonsInterface::buttonPadding = 30;
const int ButtonsInterface::xTextPadding = 20;
const int ButtonsInterface::yTextPadding = 10;
const cv::Scalar ButtonsInterface::backgroundColor = cv::Scalar(129, 129, 129);
const cv::Scalar ButtonsInterface::buttonColor = cv::Scalar(79, 79, 79);
const cv::Scalar ButtonsInterface::textColor = cv::Scalar(0, 0, 0);



/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void ButtonsInterface::OnMouseCallback(int event, int x, int y, int z, void* data)
	{
	((ButtonsInterface*)data)->OnMouseCallback(event, x, y, z);
	}

void ButtonsInterface::OnMouseCallback(int event, int x, int y, int z)
	{
    	if( event != cv::EVENT_LBUTTONDOWN )
		return;

	for(std::vector<Button>::iterator button = buttonsList.begin(); button != buttonsList.end(); button++)
		if ( x >= button->topLeftCorner.x && x <= button->bottomRightCorner.x && y>= button->topLeftCorner.y && y <= button->bottomRightCorner.y)
			{
			button->callback(button->userdataForCallback);
			return;
			}
	}


/** @} */
