/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file QualityTester.cpp
 * @date 14/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the QualityTester class.
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
#include "QualityTester.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/ply_io.h>
#include <ctime>

using namespace dfn_ci;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace FrameWrapper;
using namespace Common;

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
QualityTester::QualityTester() 
	{
	inputLeftFrame = NULL;
	inputRightFrame = NULL;
	outputPointCloud = NULL;

	inputImagesWereLoaded = false;
	outputPointCloudWasLoaded = false;
	outliersReferenceWasLoaded = false;
	measuresReferenceWasLoaded = false;
	dfnExecuted = false;
	dfnWasLoaded = false;

	SetUpMocksAndStubs();
	}

QualityTester::~QualityTester()
	{
	delete(stubFrameCache);
	delete(mockFrameConverter);

	delete(stubInverseFrameCache);
	delete(mockInverseFrameConverter);

	delete(stubCloudCache);
	delete(mockCloudConverter);
	
	DELETE_IF_NOT_NULL(inputLeftFrame);
	DELETE_IF_NOT_NULL(inputRightFrame);
	DELETE_IF_NOT_NULL(outputPointCloud);
	}

void QualityTester::SetDfn(std::string configurationFilePath, dfn_ci::StereoReconstructionInterface* dfn)
	{
	this->configurationFilePath = configurationFilePath;
	this->dfn = dfn;

	ConfigureDfn();
	dfnWasLoaded = true;
	}

void QualityTester::SetInputFilesPaths(std::string inputLeftImageFilePath, std::string inputRightImageFilePath)
	{
	this->inputLeftImageFilePath = inputLeftImageFilePath;
	this->inputRightImageFilePath = inputRightImageFilePath;

	LoadInputImage(inputLeftImageFilePath, inputLeftFrame);
	LoadInputImage(inputRightImageFilePath, inputRightFrame);
	inputImagesWereLoaded = true;
	}

void QualityTester::SetOutputFilePath(std::string outputPointCloudFilePath)
	{
	this->outputPointCloudFilePath = outputPointCloudFilePath;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile(outputPointCloudFilePath, *pclPointCloud);
	
	DELETE_IF_NOT_NULL(outputPointCloud);
	outputPointCloud = inverseCloudConverter.Convert(pclPointCloud);

	outputPointCloudWasLoaded = true;
	}

void QualityTester::SetOutliersFilePath(std::string outliersReferenceFilePath)
	{
	this->outliersReferenceFilePath = outliersReferenceFilePath;

	LoadOutliersReference();
	outliersReferenceWasLoaded = true;
	}

void QualityTester::SetMeasuresFilePath(std::string measuresReferenceFilePath)
	{
	this->measuresReferenceFilePath = measuresReferenceFilePath;

	LoadMeasuresReference();
	measuresReferenceWasLoaded = true;
	}

void QualityTester::ExecuteDfn()
	{
	ASSERT(inputImagesWereLoaded && dfnWasLoaded, "Cannot execute DFN if input images or the dfn itself are not loaded");
	dfn->leftInput(*inputLeftFrame);
	dfn->rightInput(*inputRightFrame);

	clock_t beginTime = clock();
	dfn->process();
	clock_t endTime = clock();
	float processingTime = float(endTime - beginTime) / CLOCKS_PER_SEC;
	PRINT_TO_LOG("Processing took (seconds): ", processingTime);

	DELETE_IF_NOT_NULL(outputPointCloud);
	PointCloudPtr newOutputPointCloud = NewPointCloud();
	Copy( dfn->pointcloudOutput(), *newOutputPointCloud);
	outputPointCloud = newOutputPointCloud;

	PRINT_TO_LOG("Point cloud size is", GetNumberOfPoints(*outputPointCloud));
	dfnExecuted = true;
	}

bool QualityTester::IsOutliersQualitySufficient(float outliersPercentageThreshold)
	{
	ASSERT(outputPointCloudWasLoaded && outliersReferenceWasLoaded, "Error: you have to load cloud and outliers");

	PRINT_TO_LOG("Point cloud size is", GetNumberOfPoints(*outputPointCloud));
	PRINT_TO_LOG("Number of outliers is", outliersMatrix.rows);
	
	float outlierPercentage = ((float)outliersMatrix.rows) / ( (float)GetNumberOfPoints(*outputPointCloud) );
	PRINT_TO_LOG("Outlier percentage is: ", outlierPercentage);

	bool withinOutliersThreshold = outlierPercentage < outliersPercentageThreshold;

	if (!withinOutliersThreshold)
		{
		PRINT_TO_LOG("The outliers percentage is above the threshold", outliersPercentageThreshold);
		}

	return (withinOutliersThreshold);
	}

bool QualityTester::IsCameraDistanceQualitySufficient(float cameraOperationDistance, float cameraDistanceErrorPercentage)
	{
	ASSERT(outputPointCloudWasLoaded && measuresReferenceWasLoaded, "Error: you have to load cloud and measures");
	
	float distanceToCameraErrorThreshold = cameraDistanceErrorPercentage * cameraOperationDistance;
	float cameraDistanceError = ComputeCameraDistanceError();
	bool withinCameraError = (cameraDistanceError <= distanceToCameraErrorThreshold);
	if (!withinCameraError)
		{
		PRINT_TO_LOG("The camera distance error is above the absolute value of", distanceToCameraErrorThreshold);
		}

	return withinCameraError;
	}

bool QualityTester::IsDimensionsQualitySufficient(float shapeSimilarityPercentange, float dimensionalErrorPercentage, float componentSizeThresholdPercentage)
	{
	ASSERT(outputPointCloudWasLoaded && measuresReferenceWasLoaded, "Error: you have to load cloud and measures");

	float shapeSimilarity = ComputeShapeSimilarity();
	bool validShape = (shapeSimilarity >= shapeSimilarityPercentange);
	if (!validShape)
		{
		PRINT_TO_LOG("The shape similarity is not above the value of", shapeSimilarityPercentange);
		}

	bool validDimensions = EvaluateDimensionalError(dimensionalErrorPercentage, componentSizeThresholdPercentage);
	if (!validDimensions)
		{
		PRINT_TO_LOG("The dimensional error is above the value of of", dimensionalErrorPercentage);
		}

	return (validShape && validDimensions);
	}

void QualityTester::SaveOutputPointCloud(std::string outputPointCloudFilePath)
	{
	ASSERT(dfnExecuted, "Cannot save output cloud if DFN was not executed before");
	
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclPointCloud = pointCloudConverter.Convert(outputPointCloud);

	pcl::PLYWriter writer;
	writer.write(outputPointCloudFilePath, *pclPointCloud, true);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void QualityTester::SetUpMocksAndStubs()
	{
	stubFrameCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>;
	mockFrameConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubFrameCache, mockFrameConverter);

	stubInverseFrameCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>;
	mockInverseFrameConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubInverseFrameCache, mockInverseFrameConverter);

	stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudWrapper::PointCloudConstPtr>();
	mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudWrapper::PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);
	}

void QualityTester::LoadInputImage(std::string filePath, FrameWrapper::FrameConstPtr& frame)
	{
	cv::Mat cvImage = cv::imread(filePath, CV_LOAD_IMAGE_COLOR);
	ASSERT(cvImage.cols > 0 && cvImage.rows >0, "Error: Loaded input image is empty");

	DELETE_IF_NOT_NULL(frame);	
	frame = frameConverter.Convert(cvImage);
	}

void QualityTester::LoadOutliersReference()
	{
	cv::FileStorage opencvFile(outliersReferenceFilePath, cv::FileStorage::READ);
	opencvFile["OutliersMatrix"] >> outliersMatrix;
	opencvFile.release();
	ASSERT(outliersMatrix.rows > 0 && outliersMatrix.cols == 1 && outliersMatrix.type() == CV_32SC1, "Error: reference outliers are invalid");
	}

void QualityTester::LoadMeasuresReference()
	{
	cv::Mat objectsMatrix;

	cv::FileStorage opencvFile(measuresReferenceFilePath, cv::FileStorage::READ);
	opencvFile["ObjectsMatrix"] >> objectsMatrix;
	opencvFile["PointsToCameraMatrix"] >> pointsToCameraMatrix;
	opencvFile.release();

	ASSERT( (objectsMatrix.rows == 0 || (objectsMatrix.type() == CV_32FC1 && objectsMatrix.cols == 4)), "Error in loaded file, invalid format");
	ASSERT( (pointsToCameraMatrix.rows == 0 || (pointsToCameraMatrix.type() == CV_32FC1 && pointsToCameraMatrix.cols == 2)), "Error in loaded file, invalid format");

	for(int lineIndex = 0; lineIndex < objectsMatrix.rows; lineIndex++)
		{
		int objectIndex = objectsMatrix.at<float>(lineIndex, 3);
		while(objectIndex >= objectsList.size())
			{
			Object newObject;
			objectsList.push_back(newObject);
			}	

		Line newLine;
		newLine.sourceIndex = objectsMatrix.at<float>(lineIndex, 0);
		newLine.sinkIndex = objectsMatrix.at<float>(lineIndex, 1);
		newLine.length = objectsMatrix.at<float>(lineIndex, 2);
		objectsList.at(objectIndex).push_back(newLine);
		}
	}

void QualityTester::ConfigureDfn()
	{
	dfn->setConfigurationFile(configurationFilePath);
	dfn->configure();
	}

float QualityTester::ComputeCameraDistanceError()
	{
	float cameraDistanceError = 0;
	for(int pointIndex = 0; pointIndex < pointsToCameraMatrix.rows; pointIndex++)
		{
		int32_t originalPointIndex = pointsToCameraMatrix.at<float>(pointIndex, 0);
		float measuredDistance = pointsToCameraMatrix.at<float>(pointIndex, 1);
		
		float x = GetXCoordinate(*outputPointCloud, originalPointIndex);
		float y = GetYCoordinate(*outputPointCloud, originalPointIndex);
		float z = GetZCoordinate(*outputPointCloud, originalPointIndex);
		float estimatedCameraDistance = std::sqrt(x*x + y*y + z*z);

		float error = std::abs(measuredDistance - estimatedCameraDistance);
		cameraDistanceError += error;
		}
	float averageError = cameraDistanceError / (float)pointsToCameraMatrix.rows;
	PRINT_TO_LOG("Average camera distance error is:", averageError);

	return averageError;
	}

float QualityTester::ComputeShapeSimilarity()
	{
	float totalShapeSimilarity = 0;
	for(int objectIndex = 0; objectIndex < objectsList.size(); objectIndex++)
		{
		if (objectsList.at(objectIndex).size() > 0)
			{
			totalShapeSimilarity += ComputeObjectShapeSimilarity(objectIndex);
			}
		}	
	float averageShapeSimilarity = totalShapeSimilarity / (float)objectsList.size(); 

	PRINT_TO_LOG("The average shape similarity", averageShapeSimilarity);
	return averageShapeSimilarity;
	}

bool QualityTester::EvaluateDimensionalError(float dimensionalErrorPercentage, float componentSizeThresholdPercentage)
	{
	for(int objectIndex = 0; objectIndex < objectsList.size(); objectIndex++)
		{
		float dimension = ComputeObjectDimension(objectIndex);
		for(int lineIndex = 0; lineIndex < objectsList.at(objectIndex).size(); lineIndex++)
			{
			Line& currentLine = objectsList.at(objectIndex).at(lineIndex);
			bool qualifiedLine = (currentLine.length / dimension >= componentSizeThresholdPercentage);
			if (!qualifiedLine)
				{
				continue;
				}
			
			float lineRelativeError = ComputeLineAbsoluteError(currentLine) / currentLine.length;
			if (lineRelativeError >= dimensionalErrorPercentage)
				{
				PRINT_TO_LOG("We found an qualified object component with relative error:", lineRelativeError);
				return false;
				}
			}
		}

	return true;
	}

float QualityTester::ComputeObjectDimension(int objectIndex)
	{
	float maxDimension = 0;
	for(int lineIndex = 0; lineIndex < objectsList.at(objectIndex).size(); lineIndex++)
		{
		Line& currentLine = objectsList.at(objectIndex).at(lineIndex);
		if (maxDimension < currentLine.length)
			{
			maxDimension = currentLine.length;
			}
		}
	return maxDimension;
	}

float QualityTester::ComputeLineAbsoluteError(const Line& line)
	{
	float sourceX = GetXCoordinate(*outputPointCloud, line.sourceIndex);
	float sourceY = GetYCoordinate(*outputPointCloud, line.sourceIndex);
	float sourceZ = GetZCoordinate(*outputPointCloud, line.sourceIndex);	
	float sinkX = GetXCoordinate(*outputPointCloud, line.sinkIndex);
	float sinkY = GetYCoordinate(*outputPointCloud, line.sinkIndex);
	float sinkZ = GetZCoordinate(*outputPointCloud, line.sinkIndex);	
	float differenceX = sourceX - sinkX;
	float differenceY = sourceY - sinkY;
	float differenceZ = sourceZ - sinkZ;
	float estimatedLength = std::sqrt(differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ);
	float absoluteError = std::abs(estimatedLength - line.length);
	return absoluteError;
	}

float QualityTester::ComputeObjectShapeSimilarity(int objectIndex)
	{
	float dimension = ComputeObjectDimension(objectIndex);
	
	float totalAbsoluteError = 0;
	for(int lineIndex = 0; lineIndex < objectsList.at(objectIndex).size(); lineIndex++)
		{
		Line& currentLine = objectsList.at(objectIndex).at(lineIndex);
		float lineAbsoluteError = ComputeLineAbsoluteError(currentLine);
		totalAbsoluteError += lineAbsoluteError;
		}
	float averageAbsoluteError = totalAbsoluteError / (float)objectsList.at(objectIndex).size();
	
	return (1 - (averageAbsoluteError/dimension));
	}

/** @} */
