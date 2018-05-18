/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file LinesPainter.hpp
 * @date 09/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is used to paint line keypoints on an image. It will create a window where a user can select keypoints that are supposed to lay on the same line. 
 *  By pressing keys it is possible to change the focus on the input image and save the current selected keypoints to file.
 *  
 *
 * @{
 */

#ifndef LINES_PAINTER_HPP
#define LINES_PAINTER_HPP

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
    class LinesPainter
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		/* @brief, the constructor takes as input the path to the input image, and the path to the output file in xml format. 
		*/
        	LinesPainter(std::string inputImageFilePath, std::string outputLinesFilePath);
        	~LinesPainter();

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

		typedef std::vector<Point> Line;

		static const int BASE_WINDOW_WIDTH;
		static const int BASE_WINDOW_HEIGHT;
		static const std::vector<cv::Scalar> COLORS_LIST;

		ImageZooming* imageZooming;

		cv::Mat originalImage;
		std::vector<Line> linesList;

		std::string inputImageFilePath;
		std::string outputLinesFilePath;

		void LoadImage();
		static void MouseCallback(int event, int x, int y, int flags, void* userdata);
		void MouseCallback(int event, int x, int y);
		void DrawImage();
		void ExecuteCommand(char command);
		void SaveLines();
		void LoadLines();
		int ComputeNumberOfPoints();
		void DrawLinesOnImage(cv::Mat imageToDraw);
    };

}
#endif
/* LinesPainter.hpp */
/** @} */
