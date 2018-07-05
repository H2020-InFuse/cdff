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


void ButtonsInterface::Display()
	{
	int imageColumns = buttonWidth+2*buttonPadding;
	int imageRows = buttonPadding
		+ (buttonHeight + buttonPadding) * static_cast<int>(buttonList.size());
	buttonsImage = cv::Mat(imageRows, imageColumns, CV_8UC3, backgroundColor);
	for(const auto& button: buttonList)
		{
		cv::rectangle(buttonsImage, button.topLeftCorner, button.bottomRightCorner, button.style.backgroundColor, -1, 8);
		cv::Point TextStartPoint(button.topLeftCorner.x + xTextPadding, button.bottomRightCorner.y -yTextPadding);
		cv::putText(buttonsImage, button.label, TextStartPoint, button.style.font_face, button.style.font_scale, button.style.textColor, 2, 8);
		}
	cv::imshow(windowName, buttonsImage);
	}

void ButtonsInterface::AddButton(
	std::string const &label, ButtonClickedCallback callback, const ButtonStyle style)
{
	Button button;
	button.topLeftCorner.x = buttonPadding;
	button.topLeftCorner.y = buttonPadding
		+ (buttonHeight + buttonPadding) * static_cast<int>(buttonList.size());
	button.bottomRightCorner.x = button.topLeftCorner.x + buttonWidth;
	button.bottomRightCorner.y = button.topLeftCorner.y + buttonHeight;
	button.style = style;
	button.label = label;
	button.callback = callback;
	buttonList.push_back(button);
}

void ButtonsInterface::AddButton(std::string const &label, ButtonsInterface::ButtonClickedCallback callback)
{
	AddButton(label, callback, {});
}

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

	for(const auto& button: buttonList)
		{
		if (   x >= button.topLeftCorner.x && x <= button.bottomRightCorner.x
			&& y >= button.topLeftCorner.y && y <= button.bottomRightCorner.y)
			{
			button.callback();
			return;
			}
		}
	}


/** @} */
