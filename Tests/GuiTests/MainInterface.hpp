/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MainInterface.hpp
 * @date 27/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class creates a main interface that runs as long as the exit button is not pressed. 
 * This interface is a collection of buttons each of which can be defined by a text and callback function of the type "void function()";
 * The constructor takes as input the name of the window and the size of the buttons.
 *
 * @{
 */

#ifndef MAIN_INTERFACE_HPP
#define MAIN_INTERFACE_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "ButtonsInterface.hpp"
#include <stdlib.h>
#include <string>
#include <vector>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
class MainInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:
		MainInterface(std::string windowName, int buttonWidth, int buttonHeight);
		~MainInterface() = default;

		void Run();

		void AddButton(std::string option, ButtonsInterface::on_button_clicked_cb_t callback);
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
		ButtonsInterface buttonList;
		bool shutdownNow;
		static const unsigned int refreshRate;

		void ExitCallback();


	};

#endif

/* MainInterface.hpp */
/** @} */
