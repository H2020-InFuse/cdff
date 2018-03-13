/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoCameraCalibrator.cpp
 * @date 12/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the StereoCameraCalibrator class.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "StereoCameraCalibrator.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
StereoCameraCalibrator::StereoCameraCalibrator(unsigned numberOfImages, float chessboardSquareDimension, unsigned numberOfRows, unsigned numberOfColumns,
						std::string filePath, std::string fileName, std::string fileExtension, FilesMode filesMode, std::string rightFileName)
	{
	this->numberOfImages = numberOfImages;
	this->chessboardSquareDimension = chessboardSquareDimension;
	this->filePath = filePath;
	this->fileExtension = fileExtension;
	this->filesMode = filesMode;
	this->numberOfRows = numberOfRows;
	this->numberOfColumns = numberOfColumns;

	if (filesMode == ONE_IMAGE_FILE)
		{
		this->singleFileName = fileName;
		}
	else
		{
		this->leftFileName = fileName;
		this->rightFileName = rightFileName;
		}

	ExtractChessboardCornersFromImages();
	SetUpChessboard3DCorners();
	}

StereoCameraCalibrator::~StereoCameraCalibrator()
	{

	}

void StereoCameraCalibrator::CalibrateCameras()
	{
	cv::stereoCalibrate
		(
		objectPointsList,
		leftCornersList,
		rightCornersList,
		leftCameraMatrix,
		leftDistortionCoefficients,
		rightCameraMatrix,
		rightDistortionCoefficients,
		imageSize,
		rotationMatrix,
		translationMatrix,
		essentialMatrix,
		fundamentalMatrix,
		CV_CALIB_RATIONAL_MODEL,
		cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-5)
		);
	PRINT_TO_LOG("Calibration complete", "");

	PRINT_TO_LOG("Left calibration: \n", leftCameraMatrix);
	PRINT_TO_LOG("Right calibration: \n", rightCameraMatrix);
	PRINT_TO_LOG("Left distortion: \n", leftDistortionCoefficients);
	PRINT_TO_LOG("Right distortion: \n", rightDistortionCoefficients);
	PRINT_TO_LOG("Fundamental Matrix: \n", fundamentalMatrix);
	PRINT_TO_LOG("Essential Matrix: \n", essentialMatrix);
	PRINT_TO_LOG("Translation: \n", translationMatrix);
	PRINT_TO_LOG("Rotation: \n", rotationMatrix);
	}

void StereoCameraCalibrator::RectifyCameras()
	{
	cv::stereoRectify
		(
		leftCameraMatrix,
		leftDistortionCoefficients,
		rightCameraMatrix,
		rightDistortionCoefficients,
		imageSize,
		rotationMatrix,
		translationMatrix,
		leftRectificationMatrix,
		rightRectificationMatrix,
		leftRectifiedProjectionMatrix,
		rightRectifiedProjectionMatrix,
		disparityToDepthMatrix
		);

	PRINT_TO_LOG("Rectified Left Projection Matrix \n", leftRectifiedProjectionMatrix);
	PRINT_TO_LOG("Rectified Right Projection Matrix \n", rightRectifiedProjectionMatrix);	
	PRINT_TO_LOG("Disparity To Depth Matrix \n", disparityToDepthMatrix);
	}

void StereoCameraCalibrator::ComputeUndistortionRectificationMap()
	{
	cv::Mat leftMap1, leftMap2;
	cv::initUndistortRectifyMap
		(
		leftCameraMatrix,
		leftDistortionCoefficients,
		leftRectificationMatrix,
		leftCameraMatrix,
		imageSize,
		CV_32FC1,
		leftMap1, 
		leftMap2
		);

	cv::Mat rightMap1, rightMap2;
	cv::initUndistortRectifyMap
		(
		rightCameraMatrix,
		rightDistortionCoefficients,
		rightRectificationMatrix,
		rightCameraMatrix,
		imageSize,
		CV_32FC1,
		rightMap1, 
		rightMap2
		);
	cv::FileStorage file1("../../tests/ConfigurationFiles/ImageUndistortionRectificationTransformMapsLeft.yaml", cv::FileStorage::WRITE);
	file1 << "Map1" << leftMap1 << "Map2" << leftMap2;
	file1.release();
	cv::FileStorage file2("../../tests/ConfigurationFiles/ImageUndistortionRectificationTransformMapsRight.yaml", cv::FileStorage::WRITE);
	file2 << "Map1" << rightMap1 << "Map2" << rightMap2;
	file2.release();
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

void StereoCameraCalibrator::ExtractChessboardCornersFromImages()
	{
	for(unsigned imageId=1; imageId<=numberOfImages; imageId++)
		{
		cv::Mat leftImage, rightImage;
		LoadImages(imageId, leftImage, rightImage);
		
		if (imageId == 1)
			{
			imageSize = leftImage.size();
			}
		else
			{
			ASSERT(imageSize == leftImage.size(), "Images in the set do not have same size");
			}

		std::vector<cv::Point2f> leftSingleImageCornersList, rightSingleImageCornersList;
		bool leftSuccess = cv::findChessboardCorners(leftImage, cv::Size(numberOfRows, numberOfColumns), leftSingleImageCornersList);
		bool rightSuccess = cv::findChessboardCorners(rightImage, cv::Size(numberOfRows, numberOfColumns), rightSingleImageCornersList);
		PRINT_TO_LOG("Found Corners in Left Camera", leftSuccess);
		PRINT_TO_LOG("Found Corners in Right Camera", rightSuccess);
		ASSERT(leftSuccess && rightSuccess, "Camera calibration failed");

		std::vector<cv::Point2f> sortedLeftSingleImageCornersList = SortLeftRightUpDown(leftSingleImageCornersList);
		std::vector<cv::Point2f> sortedRightSingleImageCornersList = SortLeftRightUpDown(rightSingleImageCornersList);

		leftCornersList.push_back(sortedLeftSingleImageCornersList);
		rightCornersList.push_back(sortedRightSingleImageCornersList);
		//ViewChessboard(imageId, leftImage, rightImage);
		}
	}

void StereoCameraCalibrator::LoadImages(unsigned imageIndex, cv::Mat& leftImage, cv::Mat& rightImage)
	{
	if (filesMode == ONE_IMAGE_FILE)
		{
		std::stringstream fileStream;
		fileStream<<filePath<<singleFileName<<imageIndex<<fileExtension;

		std::string fileString = fileStream.str();
		PRINT_TO_LOG("Loading File", fileString);
		cv::Mat stereoImage = cv::imread(fileString, cv::IMREAD_COLOR);

		leftImage = stereoImage( cv::Rect(0, 0, stereoImage.cols/2, stereoImage.rows) );
		rightImage = stereoImage( cv::Rect(stereoImage.cols/2, 0, stereoImage.cols/2, stereoImage.rows) );
		}
	else
		{
		std::stringstream leftFileStream, rightFileStream;
		leftFileStream<<filePath<<leftFileName<<imageIndex<<fileExtension;
		rightFileStream<<filePath<<rightFileName<<imageIndex<<fileExtension;

		std::string leftFileString = leftFileStream.str();
		PRINT_TO_LOG("Loading File", leftFileString);
		leftImage = cv::imread(leftFileString, cv::IMREAD_COLOR);

		std::string rightFileString = rightFileStream.str();
		PRINT_TO_LOG("Loading File", rightFileString);
		rightImage = cv::imread(rightFileString, cv::IMREAD_COLOR);
		}

	ASSERT(leftImage.rows > 0 && leftImage.cols > 0, "Error, left and right image should not be empty");
	ASSERT(leftImage.rows == rightImage.rows && leftImage.cols == rightImage.cols, "Error, left and right image should have same size");
	}

void StereoCameraCalibrator::SetUpChessboard3DCorners()
	{
	for(unsigned imageId = 1; imageId<=numberOfImages; imageId++)
		{
		std::vector<cv::Point3f> singleObjectPointsList;
		for(unsigned column=0; column<numberOfColumns; column++) 
			{
			for(unsigned row=0; row<numberOfRows; row++)
				{			
				cv::Point3f point((float)column*chessboardSquareDimension, (float)row*chessboardSquareDimension, 0);
				singleObjectPointsList.push_back( point );
				}
			}

		objectPointsList.push_back(singleObjectPointsList);
		}
	}

void StereoCameraCalibrator::ViewChessboard(unsigned imageIndex, cv::Mat leftImage, cv::Mat rightImage)
	{
	cv::Mat img = leftImage.clone();
	cv::drawChessboardCorners(img, cv::Size(numberOfRows, numberOfColumns), leftCornersList[imageIndex-1], true);
	cv::imshow("img", img);
	cv::waitKey(0);

	img = rightImage.clone();
	cv::drawChessboardCorners(img, cv::Size(numberOfRows, numberOfColumns), rightCornersList[imageIndex-1], true);
	cv::imshow("img", img);
	cv::waitKey(0);
	}

std::vector<cv::Point2f> StereoCameraCalibrator::SortLeftRightUpDown(std::vector<cv::Point2f>& pointsList)
	{
	/**
	This algorithm assumes that either the list is already correctly ordered or it is ordered from top-down, left-right direction;
	**/
	cv::Point2f checkPoint1 = pointsList[0];
	cv::Point2f checkPoint2 = pointsList[numberOfRows*numberOfColumns - numberOfColumns];

	float squaredDistance1ToTopLeftCorner = (checkPoint1.x * checkPoint1.x) + (checkPoint1.y * checkPoint1.y);
	float squaredDistance2ToTopLeftCorner = (checkPoint2.x * checkPoint2.x) + (checkPoint2.y * checkPoint2.y);
	bool correctOrder = squaredDistance1ToTopLeftCorner < squaredDistance2ToTopLeftCorner;

	if (correctOrder)
		{
		return pointsList;
		}
	else
		{
		return ChangeFromTopDownLeftRight(pointsList);
		}
	}

std::vector<cv::Point2f> StereoCameraCalibrator::ChangeFromTopDownLeftRight(std::vector<cv::Point2f>& pointsList)
	{
	std::vector<cv::Point2f> orderedPointsList;
	for(unsigned row = 0; row<numberOfRows; row++)
		{
		for (unsigned column = 0; column < numberOfColumns; column++)
			{
			cv::Point2f newPoint = pointsList[(numberOfColumns-column-1)*numberOfRows + row];
			orderedPointsList.push_back(newPoint);
			}
		}
	return orderedPointsList;
	}



}
/** @} */
