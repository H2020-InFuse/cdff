/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ImagesMatcher.hpp
 * @date 07/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is used to paint keypoints matches betwee two images.
 *  
 *
 * @{
 */

#ifndef IMAGES_MATCHER_HPP
#define IMAGES_MATCHER_HPP

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
    class ImagesMatcher
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		/* @brief, the constructor takes as input the paths to the input source and sink images, and the path to the output file which should have xml extension. 
		*/
        	ImagesMatcher(std::string inputSourceImageFilePath, std::string inputSinkImageFilePath, std::string outputCorrespondencesFilePath);
        	~ImagesMatcher();

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
		enum SelectionType
			{
			NOT_SELECTED,
			SELECTED_FROM_SOURCE,
			SELECTED_FROM_SINK,
			SELECTED_FROM_BOTH
			};

		struct Correspondence
			{
			int sourceX;
			int sourceY;
			int sinkX;
			int sinkY;
			SelectionType selection;
			};

		static const int BASE_WINDOW_WIDTH;
		static const int BASE_WINDOW_HEIGHT;
		static const std::vector<cv::Scalar> COLORS_LIST;

		ImageZooming* sourceImageZooming;
		ImageZooming* sinkImageZooming;

		cv::Mat originalSourceImage, originalSinkImage;
		std::vector<Correspondence> correspondencesVector;

		std::string inputSourceImageFilePath, inputSinkImageFilePath;
		std::string outputCorrespondencesFilePath;

		void LoadImages();
		static void MouseCallback(int event, int x, int y, int flags, void* userdata);
		void MouseCallback(int event, int x, int y);
		void DrawImages();
		void DrawCorrespondences(cv::Mat imageToDraw);
		void DrawLastSelectedPoint(cv::Mat imageToDraw, int nextColorIndex);
		void ExecuteCommand(char command);
		void SaveCorrespondences();
		void LoadCorrespondences();
    };

}
#endif
/* ImagesMatcher.hpp */
/** @} */
