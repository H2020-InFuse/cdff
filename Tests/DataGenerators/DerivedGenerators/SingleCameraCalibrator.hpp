/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file SingleCameraCalibrator.hpp
 * @date 23/03/2018
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

#ifndef SINGLE_CAMERA_CALIBRATOR_HPP
#define SINGLE_CAMERA_CALIBRATOR_HPP

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
    class SingleCameraCalibrator
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		enum FilesReadingMode
			{
			SEQUENTIAL_NAMES,
			NAMES_IN_LIST_FILE
			};

        	SingleCameraCalibrator(float chessboardSquareDimension, unsigned numberOfRows, unsigned numberOfColumns, 
					std::string filePath, std::string imageFilesBaseName, std::string imageFilesExtension);
        	SingleCameraCalibrator(float chessboardSquareDimension, unsigned numberOfRows, unsigned numberOfColumns, std::string filePath, std::string containerFileName);
        	~SingleCameraCalibrator();

		void CalibrateCamera();

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
		FilesReadingMode filesReadingMode;
		std::string filePath;
		std::string imageFilesExtension;
		std::string imageFilesBaseName;
		std::string containerFileName;

		unsigned numberOfImages;
		unsigned numberOfRows;
		unsigned numberOfColumns;
		float chessboardSquareDimension;

 		std::vector<std::vector<cv::Point2f> > cornersList;
		std::vector< std::vector<cv::Point3f> > objectPointsList;

		cv::Size imageSize;
		cv::Mat cameraMatrix;
		cv::Mat distortionCoefficients;
		cv::Mat rotationMatrix, translationMatrix;

		void ExtractChessboardCornersFromImages();
		bool LoadNextImage(cv::Mat& image);
		bool LoadNextImageFromListFile(cv::Mat& image);
		void SetUpChessboard3DCorners();
		void ViewChessboard(unsigned imageIndex, cv::Mat image);
		std::vector<cv::Point2f> SortLeftRightUpDown(std::vector<cv::Point2f>& pointsList);
		std::vector<cv::Point2f> ChangeFromTopDownLeftRight(std::vector<cv::Point2f>& pointsList);

		void PrintCameraMatrix(std::string message, cv::Mat cameraMatrix);
		void PrintDistortionCoefficients(std::string message, cv::Mat distortionVector);
		
    };

}
#endif
/* SingleCameraCalibrator.hpp */
/** @} */
