/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ButtonsInterface.hpp
 * @date 24/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * This class creates a buttons interface by using basic GUI functions from OpenCV library. 
 * The class constructor needs a windowName, and the dimensions of the buttons.
 * A button is defined by a text and a callback function of the type "void function()".
 *
 * @{
 */

#ifndef BUTTONS_INTERFACE_HPP
#define BUTTONS_INTERFACE_HPP


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
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
class ButtonsInterface
	{
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
	public:

		using on_button_clicked_cb_t = std::function<void()>;

		/**
		 * The `ButtonStyle` struct is used to control the appearance of
		 * Buttons.
		 * @see ButtonsInterface::AddButton
		 */
		struct ButtonStyle
		{
			cv::Scalar backgroundColor = cv::Scalar(79, 79, 79);
			cv::Scalar textColor = cv::Scalar(0, 0, 0);

			int font_face = cv::FONT_HERSHEY_SIMPLEX;
			double font_scale = 0.6;
		};

		ButtonsInterface(std::string windowName, int buttonWidth, int buttonHeight);
		~ButtonsInterface() = default;

		void Display();

		/**
		 * Add a button to this `ButtonsInterface`
		 *
		 * @param label The text of the button
		 * @param callback The callback function to call when the button is clicked
		 * @param style The colours used on the button
		 */
		void AddButton(
			std::string const &label,
			on_button_clicked_cb_t callback,
			const ButtonStyle style
		);

		/**
		* Add a button to this `ButtonsInterface` with default styling
		*
		* @param label The text of the button
		* @param callback The callback function to call when the button is clicked
		*/
		void AddButton(
			std::string const &label,
			on_button_clicked_cb_t callback
		);

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
		typedef void (*Callback)(void*);

		struct Button
			{
			cv::Point topLeftCorner;
			cv::Point bottomRightCorner;
			ButtonStyle style;
			std::string label;
			on_button_clicked_cb_t callback;
			};

		const std::string windowName;
		std::vector<Button> buttonList;
		cv::Mat buttonsImage;
		int buttonWidth;
		int buttonHeight;

		// Button Appearance

		/// The amount of space (in pixels) between the buttons and its neighbours
		/// or the edge of the canvas.
		int buttonPadding = 30;

		/// The amount of free space (in pixels) between the edge of the button
		/// and its label text along the _x_ direction
		int xTextPadding = 20;

	/// The amount of free space (in pixels) between the edge of the button
	/// and its label text along the _y_ direction
		int yTextPadding = 10;

		/// The colour of the background of the image onto which the buttons
		/// will be rendered.
		// The default background colour is the same as the background colour of
		// OpenCV windows.
		cv::Scalar backgroundColor = cv::Scalar(215, 217, 220);

		static void OnMouseCallback(int event, int x, int y, int z, void* data);
		void OnMouseCallback(int event, int x, int y, int z);
	};

#endif

/* ButtonsInterface.hpp */
/** @} */
