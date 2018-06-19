/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file MultipleImagesMatcher.hpp
 * @date 19/06/2018
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

#ifndef MULTIPLE_IMAGES_MATCHER_HPP
#define MULTIPLE_IMAGES_MATCHER_HPP

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
    class MultipleImagesMatcher
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		/* @brief, the constructor takes as input the paths to the input images, and the path to the output file which should have xml extension. 
		*/
        	MultipleImagesMatcher(std::vector<std::string> inputImageFilePathList, std::string outputCorrespondencesFilePath);
        	~MultipleImagesMatcher();

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

		std::vector<ImageZooming*> imageZoomingList;
		std::vector<cv::Mat> originalImageList;
		int currentSourceIndex;
		int currentSinkIndex;

		std::vector< std::vector<Correspondence> > correspondenceVectorList;

		std::vector<std::string> inputImageFilePathList;
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

		std::vector<Correspondence>& GetCurrentCorrespondenceVector();
    };

}
#endif
/* MultipleImagesMatcher.hpp */
/** @} */
