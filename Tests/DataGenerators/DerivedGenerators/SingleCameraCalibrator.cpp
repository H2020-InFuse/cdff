/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SingleCameraCalibrator.cpp
 * @date 23/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the SingleCameraCalibrator class.
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
#include "SingleCameraCalibrator.hpp"
#include <stdlib.h>
#include <time.h>
#include <ctime>
#include <Errors/Assert.hpp>
#include <iomanip>  
#include <fstream>
#include <string>
#include <sstream>
#include <boost/algorithm/string.hpp>

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
SingleCameraCalibrator::SingleCameraCalibrator(float chessboardSquareDimension, unsigned numberOfRows, unsigned numberOfColumns, 
		std::string filePath, std::string imageFilesBaseName, std::string imageFilesExtension)
	{
	PRINT_TO_LOG("seq", "creation");
	this->chessboardSquareDimension = chessboardSquareDimension;
	this->numberOfRows = numberOfRows;
	this->numberOfColumns = numberOfColumns;
	this->filePath = filePath;
	this->imageFilesBaseName = imageFilesBaseName;
	this->imageFilesExtension = imageFilesExtension;
	this->filesReadingMode = SEQUENTIAL_NAMES;

	ExtractChessboardCornersFromImages();
	SetUpChessboard3DCorners();
	}

SingleCameraCalibrator::SingleCameraCalibrator(float chessboardSquareDimension, unsigned numberOfRows, unsigned numberOfColumns, std::string filePath, std::string containerFileName)
	{
	PRINT_TO_LOG("IN FILES", "creation");
	this->chessboardSquareDimension = chessboardSquareDimension;
	this->numberOfRows = numberOfRows;
	this->numberOfColumns = numberOfColumns;
	this->filePath = filePath;
	this->containerFileName = containerFileName;
	this->filesReadingMode = NAMES_IN_LIST_FILE;

	ExtractChessboardCornersFromImages();
	SetUpChessboard3DCorners();
	}

SingleCameraCalibrator::~SingleCameraCalibrator()
	{

	}

void SingleCameraCalibrator::CalibrateCamera()
	{
	PRINT_TO_LOG("Using this number of images: \n", numberOfImages);
	PRINT_TO_LOG("Calibrating Images with Size: \n", imageSize);

	cv::calibrateCamera
		(
		objectPointsList,
		cornersList,
		imageSize,
		cameraMatrix,
		distortionCoefficients,
		rotationMatrix,
		translationMatrix,
		CV_CALIB_RATIONAL_MODEL,
		cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 1000, 1e-5)
		);
	PRINT_TO_LOG("Calibration complete", "");

	PrintCameraMatrix("Camera Matrix \n", cameraMatrix);
	PrintDistortionCoefficients("Distortion Matrix \n", distortionCoefficients);
	PRINT_TO_LOG("Translation: \n", translationMatrix);
	PRINT_TO_LOG("Rotation: \n", rotationMatrix);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */

void SingleCameraCalibrator::ExtractChessboardCornersFromImages()
	{
	numberOfImages = 0;
	cv::Mat image;
	while ( LoadNextImage(image) )
		{
		numberOfImages++;
		
		if (numberOfImages == 1)
			{
			imageSize = image.size();
			}
		else
			{
			ASSERT(imageSize == image.size(), "Images in the set do not have same size");
			}

		std::vector<cv::Point2f> singleImageCornersList;
		bool success = cv::findChessboardCorners(image, cv::Size(numberOfRows, numberOfColumns), singleImageCornersList,
			cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
		PRINT_TO_LOG("Found Corners in Camera", success);
		if (success)
			{
			std::vector<cv::Point2f> sortedSingleImageCornersList = SortLeftRightUpDown(singleImageCornersList);
			cornersList.push_back(sortedSingleImageCornersList);
			//ViewChessboard(numberOfImages, image);
			}
		else
			{
			numberOfImages--;
			}
		}
	}

bool SingleCameraCalibrator::LoadNextImage(cv::Mat& image)
	{
	if (filesReadingMode == SEQUENTIAL_NAMES)
		{
		std::stringstream fileStream;
		fileStream<<filePath<<imageFilesBaseName<<(numberOfImages+1)<<imageFilesExtension;

		std::string fileString = fileStream.str();
		PRINT_TO_LOG("Loading File", fileString);
		image = cv::imread(fileString, cv::IMREAD_COLOR);

		return ( image.cols > 0 && image.rows > 0);
		}
	else
		{
		return LoadNextImageFromListFile(image);
		}
	}

bool SingleCameraCalibrator::LoadNextImageFromListFile(cv::Mat& image)
	{
	static std::ifstream containerFile;
	static bool firstTime = true;
	static bool lastTime = false;
	ASSERT(!lastTime, "Error LoadNextImageFromListFile called after end of input. Only one input reading is allowed for class instance!");
	
	std::string line;
	if (firstTime)
		{
		std::stringstream fullContainerName;
		fullContainerName << filePath << "/" << containerFileName;

		containerFile.open(fullContainerName.str().c_str());
		ASSERT(containerFile.good(), "Error, container file not found or corrupted");
		firstTime = false;

		// The first three lines have no useful information
		std::getline(containerFile, line);
		std::getline(containerFile, line);
		std::getline(containerFile, line);
		}

	if (!std::getline(containerFile, line))
		{
		containerFile.close();
		lastTime = true;
		return false;
		}	

	std::vector<std::string> stringsList;
	boost::split(stringsList, line, boost::is_any_of(" "));
	std::stringstream imageFileStream;
	imageFileStream << filePath << "/" <<stringsList.at(1);

	std::string imageFileString = imageFileStream.str();
	PRINT_TO_LOG("Loading File", imageFileString);
	image = cv::imread(imageFileString, cv::IMREAD_COLOR);
	return true;
	}

void SingleCameraCalibrator::SetUpChessboard3DCorners()
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

void SingleCameraCalibrator::ViewChessboard(unsigned imageIndex, cv::Mat image)
	{
	cv::Mat img = image.clone();
	cv::drawChessboardCorners(img, cv::Size(numberOfRows, numberOfColumns), cornersList[imageIndex-1], true);
	cv::imshow("img", img);
	cv::waitKey(0);
	}

std::vector<cv::Point2f> SingleCameraCalibrator::SortLeftRightUpDown(std::vector<cv::Point2f>& pointsList)
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

std::vector<cv::Point2f> SingleCameraCalibrator::ChangeFromTopDownLeftRight(std::vector<cv::Point2f>& pointsList)
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

void SingleCameraCalibrator::PrintCameraMatrix(std::string message, cv::Mat cameraMatrix)
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

void SingleCameraCalibrator::PrintDistortionCoefficients(std::string message, cv::Mat distortionVector)
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



}
/** @} */
