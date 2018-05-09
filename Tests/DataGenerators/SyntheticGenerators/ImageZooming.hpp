/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ImageZooming.hpp
 * @date 07/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is used to zoom in a part of an image. The purpose of the class is to highlight one part of an image, and to scale it up or down.
 *  
 *
 * @{
 */

#ifndef IMAGE_ZOOMING_HPP
#define IMAGE_ZOOMING_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace DataGenerators {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class ImageZooming
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
        	ImageZooming(int imageWidth, int imageHeight, int displayWidth, int displayHeight);
        	~ImageZooming();

		/* @brief the method extracts a part of the input image according to the current zooming settings
		*
		* @param inputImage, the image whose part has to be extracted
		*/
		cv::Mat ExtractZoomedWindow(cv::Mat inputImage);
		
		/* @brief the method extracts a part of the input image with an added overlay according to the current zooming settings
		*
		* @param inputImage, the image whose part has to be extracted, it has to be of type CV_8UC3
		* @param overlayImage, the overlay image that has to be written on top of the image. This image has an alpha channel that dictates how much visible it will be, it has type CV_8UC4.
		*/
		cv::Mat ExtractZoomedWindow(cv::Mat inputImage, cv::Mat overlayImage);

		/* @brief the method extracts a part of the input image with an added overlay according to the current zooming settings
		*
		* @param inputImage, the image whose part has to be extracted.
		* @param overlayImage, the overlay image that has to be written on top of the image.
		* @param alpha, the alpha value that dictates how much the overlay is visible on top of the input image, it is a number from 0 to 1.
		*/
		cv::Mat ExtractZoomedWindow(cv::Mat inputImage, cv::Mat overlayImage, float alpha);

		/* @brief the method increases the scale of the extracted images
		*/		
		void ZoomIn();

		/* @brief the method reduces the scale of the extracted images
		*/		
		void ZoomOut();

		/* @brief the method shifts the focus of the extracted images to the left
		*/		
		void MoveFocusLeft();

		/* @brief the method shifts the focus of the extracted images to the right
		*/		
		void MoveFocusRight();

		/* @brief the method shifts the focus of the extracted images to the top
		*/		
		void MoveFocusUp();

		/* @brief the method shifts the focus of the extracted images to the bottom
		*/		
		void MoveFocusDown();

		/* @brief the method converts the coordinate of a pixel on the extracted image, to the coordinates of the corresponding pixel in the input image.
		*
		* @param (windowX, windowY), input coordinates on the extracted image;
 		* @param (imageX, imageY), output coordinates on the input image;
		* @output valid, the boolwan value determines whether the output coordinates point is really within the input image.If the output is false (imageX, imageY) is meaningless.
		*/		
		bool WindowToImagePixel(int windowX, int windowY, int& imageX, int& imageY); 

		/* @brief the method converts the coordinate of a pixel on the input image, to the coordinates of the corresponding pixel in the extracted image.
		*
 		* @param (imageX, imageY), input coordinates on the input image;
		* @param (windowX, windowY), output coordinates on the extracted image;
		* @output valid, the boolean value determines whether the input image coordinates have really a corresponding pixel. If the output is false (windowX, windowY) is meaningless.
		*/	
		bool ImageToWindowPixel(int imageX, int imageY, int& windowX, int& windowY); 

		/* @brief the method determines whethe the image coordinates are visible in the extracted image.
		*
 		* @param (imageX, imageY), input coordinates on the input image;
		* @output valid, the boolean value determines whether the input image coordinates have really a corresponding pixel.
		*/
		bool PixelIsWithinZoomedWindow(int imageX, int imageY);
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
		static const int MAXIMUM_SCALE;		

		int imageWidth, imageHeight;
		int displayWidth, displayHeight;
		int zoomWindowWidth, zoomWindowHeight;

		int scale;
		int offsetWidth;
		int offsetHeight;

		void DrawZoomPixel(cv::Mat zoomWindow, int scaledRowIndex, int scaledColumnIndex, const cv::Vec3b& originalPixel);
    };

}
#endif
/* ImageZooming.hpp */
/** @} */
