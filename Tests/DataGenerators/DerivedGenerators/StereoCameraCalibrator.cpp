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
#include <iomanip>  

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
	PRINT_TO_LOG("Calibrating Images with Size: \n", imageSize);
 
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

	PrintCameraMatrix("Left Camera Matrix \n", leftCameraMatrix);
	PrintCameraMatrix("Right Camera Matrix \n", rightCameraMatrix);
	PrintDistortionCoefficients("Left Distortion Matrix \n", leftDistortionCoefficients);
	PrintDistortionCoefficients("Right Distortion Matrix \n", rightDistortionCoefficients);
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

	PrintRectificationMatrix("Left Rectification Matrix \n", leftRectificationMatrix);
	PrintRectificationMatrix("Right Rectification Matrix \n", rightRectificationMatrix);
	PRINT_TO_LOG("Rectified Left Projection Matrix \n", leftRectifiedProjectionMatrix);
	PRINT_TO_LOG("Rectified Right Projection Matrix \n", rightRectifiedProjectionMatrix);	
	PrintDisparityToDepthMatrix(disparityToDepthMatrix);

	double baseline = std::sqrt(
			translationMatrix.at<double>(0,0)*translationMatrix.at<double>(0,0) +
			translationMatrix.at<double>(1,0)*translationMatrix.at<double>(1,0) +
			translationMatrix.at<double>(2,0)*translationMatrix.at<double>(2,0)
			);
	PRINT_TO_LOG("Baseline \n", baseline);
	}

void StereoCameraCalibrator::ComputeUndistortionRectificationMap()
	{
	cv::Mat leftMap1, leftMap2;
	cv::initUndistortRectifyMap
		(
		leftCameraMatrix,
		leftDistortionCoefficients,
		leftRectificationMatrix,
		cv::getOptimalNewCameraMatrix(leftCameraMatrix, leftDistortionCoefficients, imageSize, 1),
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
		cv::getOptimalNewCameraMatrix(rightCameraMatrix, rightDistortionCoefficients, imageSize, 1),
		imageSize,
		CV_32FC1,
		rightMap1, 
		rightMap2
		);
	PRINT_TO_LOG("Wrinting left map to file: ", leftMapFilePath);
	cv::FileStorage file1(leftMapFilePath, cv::FileStorage::WRITE);
	file1 << "Map1" << leftMap1 << "Map2" << leftMap2;
	file1.release();

	PRINT_TO_LOG("Wrinting right map to file: ", rightMapFilePath);
	cv::FileStorage file2(rightMapFilePath, cv::FileStorage::WRITE);
	file2 << "Map1" << rightMap1 << "Map2" << rightMap2;
	file2.release();
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Variables
 *
 * --------------------------------------------------------------------------
 */
const std::string StereoCameraCalibrator::leftMapFilePath = "../tests/ConfigurationFiles/DFNs/ImageFiltering/ImageUndistortionRectificationTransformMapsLeft.yaml";
const std::string StereoCameraCalibrator::rightMapFilePath = "../tests/ConfigurationFiles/DFNs/ImageFiltering/ImageUndistortionRectificationTransformMapsRight.yaml";

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
		bool leftSuccess = cv::findChessboardCorners(leftImage, cv::Size(numberOfRows, numberOfColumns), leftSingleImageCornersList,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		bool rightSuccess = cv::findChessboardCorners(rightImage, cv::Size(numberOfRows, numberOfColumns), rightSingleImageCornersList,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		PRINT_TO_LOG("Found Corners in Left Camera", leftSuccess);
		PRINT_TO_LOG("Found Corners in Right Camera", rightSuccess);
		ASSERT(leftSuccess && rightSuccess, "Camera calibration failed");

		std::vector<cv::Point2f> sortedLeftSingleImageCornersList = SortLeftRightUpDown(leftSingleImageCornersList);
		std::vector<cv::Point2f> sortedRightSingleImageCornersList = SortLeftRightUpDown(rightSingleImageCornersList);

		leftCornersList.push_back(sortedLeftSingleImageCornersList);
		rightCornersList.push_back(sortedRightSingleImageCornersList);
		ViewChessboard(imageId, leftImage, rightImage);
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
	/** If the chessboard is not square the ordering is guaranteed already **/
	if (numberOfRows != numberOfColumns)
		{
		return pointsList;	
		}
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

void StereoCameraCalibrator::PrintCameraMatrix(std::string message, cv::Mat cameraMatrix)
	{
	std::stringstream messageStream;
	messageStream<<std::setprecision(14);
	messageStream<<"   FocalLengthX: " << cameraMatrix.at<double>(0,0) <<"\n";
	messageStream<<"    FocalLengthY: " << cameraMatrix.at<double>(1,1) <<"\n";
	messageStream<<"    PrinciplePointX: " << cameraMatrix.at<double>(0,2) <<"\n";
	messageStream<<"    PrinciplePointY: " << cameraMatrix.at<double>(1,2) <<"\n";
	std::string messageString = messageStream.str();
	PRINT_TO_LOG(message, messageString);
	}

void StereoCameraCalibrator::PrintDistortionCoefficients(std::string message, cv::Mat distortionVector)
	{
	std::stringstream messageStream;
	messageStream<<std::setprecision(14);
	messageStream<<"   UseK3: true \n";
	messageStream<<"    UseK4ToK6: true \n";
	messageStream<<"    K1: " << distortionVector.at<double>(0, 0) << "\n";
	messageStream<<"    K2: " << distortionVector.at<double>(0, 1) << "\n";
	messageStream<<"    P1: " << distortionVector.at<double>(0, 2) << "\n";
	messageStream<<"    P2: " << distortionVector.at<double>(0, 3) << "\n";
	messageStream<<"    K3: " << distortionVector.at<double>(0, 4) << "\n";
	messageStream<<"    K4: " << distortionVector.at<double>(0, 5) << "\n";
	messageStream<<"    K5: " << distortionVector.at<double>(0, 6) << "\n";
	messageStream<<"    K6: " << distortionVector.at<double>(0, 7) << "\n";
	std::string messageString = messageStream.str();
	PRINT_TO_LOG(message, messageString);
	}

void StereoCameraCalibrator::PrintRectificationMatrix(std::string message, cv::Mat rectificationMatrix)
	{
	std::stringstream messageStream;
	messageStream<<std::setprecision(14);
	for(unsigned row = 0; row < 3; row++)
		{
		for(unsigned column = 0; column < 3; column++)
			{
			if (row == 0 && column == 0)
				{
				messageStream<<"   Element_" << row << "_" << column <<": " << rectificationMatrix.at<double>(row, column) << "\n";
				}
			else
				{
				messageStream<<"    Element_" << row << "_" << column <<": " << rectificationMatrix.at<double>(row, column) << "\n";
				}
			}
		}
	std::string messageString = messageStream.str();
	PRINT_TO_LOG(message, messageString);
	}

void StereoCameraCalibrator::PrintDisparityToDepthMatrix(cv::Mat disparityToDepthMatrix)
	{
	std::stringstream messageStream;
	messageStream<<std::setprecision(14);
	for(unsigned row = 0; row < 4; row++)
		{
		for(unsigned column = 0; column < 4; column++)
			{
			if (row == 0 && column == 0)
				{
				messageStream<<"   Element_" << row << "_" << column <<": " << disparityToDepthMatrix.at<double>(row, column) << "\n";
				}
			else
				{
				messageStream<<"    Element_" << row << "_" << column <<": " << disparityToDepthMatrix.at<double>(row, column) << "\n";
				}
			}
		}
	std::string messageString = messageStream.str();
	PRINT_TO_LOG("DisparityToDepthMatrix: \n", messageString);
	}



}
/** @} */
