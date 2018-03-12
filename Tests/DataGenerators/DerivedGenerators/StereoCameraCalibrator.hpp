/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StereoCameraCalibrator.hpp
 * @date 12/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 *  This class is used to extract camera parameters from a set of chessboard images.
 *  
 *
 * @{
 */

#ifndef STEREO_CAMERA_CALIBRATOR_HPP
#define STEREO_CAMERA_CALIBRATOR_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>


namespace DataGenerators {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class StereoCameraCalibrator
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		enum FilesMode
			{
			ONE_IMAGE_FILE,
			TWO_IMAGE_FILES
			};

        	StereoCameraCalibrator(unsigned numberOfImages, float chessboardSquareDimension, unsigned numberOfRows, unsigned numberOfColumns, 
					std::string filePath, std::string fileName, std::string fileExtension = ".jpg", FilesMode filesMode = TWO_IMAGE_FILES, std::string rightFileName = "Right");
        	~StereoCameraCalibrator();

		void CalibrateCameras();
		void RectifyCameras();
		void ComputeUndistortionRectificationMap();

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
		FilesMode filesMode;
		std::string filePath;
		std::string fileExtension;
		std::string leftFileName;
		std::string rightFileName;
		std::string singleFileName;

		unsigned numberOfImages;
		unsigned numberOfRows;
		unsigned numberOfColumns;
		float chessboardSquareDimension;

 		std::vector<std::vector<cv::Point2f> > leftCornersList;
 		std::vector<std::vector<cv::Point2f> > rightCornersList;
		std::vector< std::vector<cv::Point3f> > objectPointsList;

		cv::Size imageSize;
		cv::Mat leftCameraMatrix, rightCameraMatrix;
		cv::Mat leftDistortionCoefficients, rightDistortionCoefficients;
		cv::Mat rotationMatrix, translationMatrix, essentialMatrix, fundamentalMatrix;
		cv::Mat leftRectificationMatrix, rightRectificationMatrix, leftRectifiedProjectionMatrix, rightRectifiedProjectionMatrix;
		cv::Mat disparityToDepthMatrix;

		void ExtractChessboardCornersFromImages();
		void LoadImages(unsigned imageIndex, cv::Mat& leftImage, cv::Mat& rightImage);
		void SetUpChessboard3DCorners();
		
    };

}
#endif
/* StereoCameraCalibrator.hpp */
/** @} */
