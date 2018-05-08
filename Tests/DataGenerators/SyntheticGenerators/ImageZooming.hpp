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
 *  This class is used to zoom in a part of an image.
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

		cv::Mat ExtractZoomedWindow(cv::Mat inputImage);
		cv::Mat ExtractZoomedWindow(cv::Mat inputImage, cv::Mat overlayImage);
		cv::Mat ExtractZoomedWindow(cv::Mat inputImage, cv::Mat overlayImage, float alpha);
		void ZoomIn();
		void ZoomOut();
		void MoveFocusLeft();
		void MoveFocusRight();
		void MoveFocusUp();
		void MoveFocusDown();
		void WindowToImagePixel(int windowX, int windowY, int& imageX, int& imageY); 
		bool ImageToWindowPixel(int imageX, int imageY, int& windowX, int& windowY); 
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
