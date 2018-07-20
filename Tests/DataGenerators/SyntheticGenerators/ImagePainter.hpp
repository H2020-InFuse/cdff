/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ImagePainter.hpp
 * @date 04/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is used to paint keypoints on an image. It will create a window where a user can select key-points that will change color. By pressing keys it is possible to change the focus on the
 *  input image and save the current selected keypoints to file.
 *  
 *
 * @{
 */

#ifndef IMAGE_PAINTER_HPP
#define IMAGE_PAINTER_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "ImageZooming.hpp"
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
    class ImagePainter
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		/* @brief, the constructor takes as input the path to the input image, and the path to the output image file. Loseless png and ppm formats are preferred in output. 
		*/
        	ImagePainter(std::string inputImageFilePath, std::string outputKeypointsFilePath);
        	~ImagePainter();

		/* @brief, this method is called to start the window engine. 
		*/
		void Run();
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
		struct Point
			{
			int x;
			int y;
			};

		static const int BASE_WINDOW_WIDTH;
		static const int BASE_WINDOW_HEIGHT;
		static const std::vector<cv::Scalar> COLORS_LIST;

		ImageZooming* imageZooming;

		cv::Mat originalImage;
		std::vector<Point> keypointsList;
		bool boxSelection;
		Point topLeftCorner;
		bool topLeftCornerSelected;

		std::string inputImageFilePath;
		std::string outputKeypointsFilePath;

		void LoadImage();
		static void MouseCallback(int event, int x, int y, int flags, void* userdata);
		void MouseCallback(int event, int x, int y);
		void DrawImage();
		void DrawKeypoints(cv::Mat imageToDraw);
		void ExecuteCommand(char command);
		void SaveKeypointsImage();
		void LoadKeypointsImage();
    };

}
#endif
/* ImagePainter.hpp */
/** @} */
