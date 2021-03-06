/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionExecutor.cpp
 * @date 16/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 *
 * Implementation of the ReconstructionExecutor class.
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
#include "ReconstructionExecutor.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <pcl/io/ply_io.h>
#include <ctime>

using namespace CDFF::DFPC;
using namespace Converters;
using namespace PointCloudWrapper;
using namespace PoseWrapper;
using namespace FrameWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace SupportTypes;

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
ReconstructionExecutor::ReconstructionExecutor()
	{
	inputLeftFrame = NULL;
	inputRightFrame = NULL;
	outputPointCloud = NULL;
	outputCameraPose = NULL;

	inputImagesWereLoaded = false;
	outputPointCloudWasLoaded = false;
	outliersReferenceWasLoaded = false;
	measuresReferenceWasLoaded = false;
	dfpcExecuted = false;
	dfpcWasLoaded = false;

	outputSuccess = false;
	dfpc = NULL;
	}

ReconstructionExecutor::~ReconstructionExecutor()
	{
	DELETE_IF_NOT_NULL(inputLeftFrame);
	DELETE_IF_NOT_NULL(inputRightFrame);
	DELETE_IF_NOT_NULL(outputPointCloud);
	DELETE_IF_NOT_NULL(outputCameraPose);
	}

void ReconstructionExecutor::SetDfpc(const std::string& configurationFilePath, CDFF::DFPC::Reconstruction3DInterface* dfpc)
	{
	this->configurationFilePath = configurationFilePath;
	this->dfpc = dfpc;

	ConfigureDfpc();
	dfpcWasLoaded = true;
	}

void ReconstructionExecutor::SetInputFilesPaths(const std::string& inputImagesFolder, const std::string& inputImagesListFileName)
	{
	this->inputImagesFolder = inputImagesFolder;
	this->inputImagesListFileName = inputImagesListFileName;

	LoadInputImagesList();
	inputImagesWereLoaded = true;
	}

void ReconstructionExecutor::SetOutputFilePath(const std::string& outputPointCloudFilePath)
	{
	this->outputPointCloudFilePath = outputPointCloudFilePath;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile(outputPointCloudFilePath, *pclPointCloud);

	DELETE_IF_NOT_NULL(outputPointCloud);
	outputPointCloud = inverseCloudConverter.Convert(pclPointCloud);

	outputPointCloudWasLoaded = true;
	}

void ReconstructionExecutor::SetOutliersFilePath(const std::string& outliersReferenceFilePath)
	{
	this->outliersReferenceFilePath = outliersReferenceFilePath;

	LoadOutliersReference();
	outliersReferenceWasLoaded = true;
	}

void ReconstructionExecutor::SetMeasuresFilePath(const std::string& measuresReferenceFilePath)
	{
	this->measuresReferenceFilePath = measuresReferenceFilePath;

	LoadMeasuresReference();
	measuresReferenceWasLoaded = true;
	}

void ReconstructionExecutor::ExecuteDfpc(const std::string& transformFilePath)
	{
	ASSERT(inputImagesWereLoaded && dfpcWasLoaded, "Cannot execute DFPC if input images or the DFPC itself are not loaded");
	ASSERT(leftImageFileNamesList.size() == rightImageFileNamesList.size(), "Left images list and right images list do not have same dimensions");

	int successCounter = 0;
	float processingTime = 0;
	for(int imageIndex = 0; imageIndex < leftImageFileNamesList.size(); imageIndex++)
		{
		std::stringstream leftImageFilePath, rightImageFilePath;
		leftImageFilePath << inputImagesFolder << "/" << leftImageFileNamesList.at(imageIndex);
		rightImageFilePath << inputImagesFolder << "/" << rightImageFileNamesList.at(imageIndex);
		LoadInputImage(leftImageFilePath.str(), inputLeftFrame);
		LoadInputImage(rightImageFilePath.str(), inputRightFrame);

		dfpc->leftImageInput(*inputLeftFrame);
		dfpc->rightImageInput(*inputRightFrame);

		clock_t beginTime = clock();
		dfpc->run();
		clock_t endTime = clock();
		processingTime += float(endTime - beginTime) / CLOCKS_PER_SEC;

		DELETE_IF_NOT_NULL(outputPointCloud);
		PointCloudPtr newOutputPointCloud = NewPointCloud();
		Copy( dfpc->pointCloudOutput(), *newOutputPointCloud);
		outputPointCloud = newOutputPointCloud;

		DELETE_IF_NOT_NULL(outputCameraPose);
		Pose3DPtr newOutputCameraPose = NewPose3D();
		Copy( dfpc->poseOutput(), *newOutputCameraPose);
		outputCameraPose = newOutputCameraPose;

		SaveTransform(transformFilePath);

		outputSuccess = dfpc->successOutput();
		successCounter = (outputSuccess ? successCounter+1 : successCounter);
		}

	PRINT_TO_LOG("Processing took (seconds): ", processingTime);
	PRINT_TO_LOG("The reconstruction was successful on this number of images:", successCounter);
	PRINT_TO_LOG("The final point cloud has size:", GetNumberOfPoints(*outputPointCloud));
	PRINT_TO_LOG("The final pose of the camera is:", ToString(*outputCameraPose));
	dfpcExecuted = true;
	}

bool ReconstructionExecutor::IsOutliersQualitySufficient(float outliersPercentageThreshold)
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

bool ReconstructionExecutor::IsCameraDistanceQualitySufficient(float cameraOperationDistance, float cameraDistanceErrorPercentage)
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

bool ReconstructionExecutor::IsDimensionsQualitySufficient(float shapeSimilarityPercentange, float dimensionalErrorPercentage, float componentSizeThresholdPercentage)
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

void ReconstructionExecutor::SaveOutputPointCloud(const std::string& outputPointCloudFilePath)
	{
	ASSERT(dfpcExecuted, "Cannot save output cloud if DFPC was not executed before");

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
void ReconstructionExecutor::LoadInputImage(const std::string& filePath, FrameWrapper::FrameConstPtr& frame)
	{
	cv::Mat cvImage = cv::imread(filePath, CV_LOAD_IMAGE_COLOR);
	ASSERT(cvImage.cols > 0 && cvImage.rows >0, "Error: Loaded input image is empty");

	DELETE_IF_NOT_NULL(frame);
	frame = frameConverter.Convert(cvImage);
	}

void ReconstructionExecutor::LoadInputImagesList()
	{
	std::stringstream imagesListFilePath;
	imagesListFilePath << inputImagesFolder << "/" << inputImagesListFileName;
	std::ifstream imagesListFile(imagesListFilePath.str().c_str());
	ASSERT(imagesListFile.good(), "Error it was not possible to open the images list file");

	std::string line;
	std::getline(imagesListFile, line);
	std::getline(imagesListFile, line);
	std::getline(imagesListFile, line);
	while (std::getline(imagesListFile, line))
		{
		std::vector<std::string> stringsList;
		boost::split(stringsList, line, boost::is_any_of(" "));
		ASSERT(stringsList.size() == 3, "Error reading file, bad line");

		leftImageFileNamesList.push_back( std::string(stringsList.at(1)) );
		rightImageFileNamesList.push_back( std::string(stringsList.at(2)) );
		}

	imagesListFile.close();
	}

void ReconstructionExecutor::LoadOutliersReference()
	{
	cv::FileStorage opencvFile(outliersReferenceFilePath, cv::FileStorage::READ);
	opencvFile["OutliersMatrix"] >> outliersMatrix;
	opencvFile.release();
	ASSERT(outliersMatrix.rows > 0 && outliersMatrix.cols == 1 && outliersMatrix.type() == CV_32SC1, "Error: reference outliers are invalid");
	}

void ReconstructionExecutor::LoadMeasuresReference()
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

void ReconstructionExecutor::ConfigureDfpc()
	{
	dfpc->setConfigurationFile(configurationFilePath);
	dfpc->setup();
	}

float ReconstructionExecutor::ComputeCameraDistanceError()
	{
	ASSERT(pointsToCameraMatrix.rows > 0, "No measures available in input file");
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

float ReconstructionExecutor::ComputeShapeSimilarity()
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

bool ReconstructionExecutor::EvaluateDimensionalError(float dimensionalErrorPercentage, float componentSizeThresholdPercentage)
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

float ReconstructionExecutor::ComputeObjectDimension(int objectIndex)
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

float ReconstructionExecutor::ComputeLineAbsoluteError(const Line& line)
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

float ReconstructionExecutor::ComputeObjectShapeSimilarity(int objectIndex)
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

void ReconstructionExecutor::SaveTransform(const std::string& transformFilePath)
	{
	if (transformFilePath == "")
		{
		return;
		}
	static unsigned index = 0;
	std::ofstream file;

	if (index == 0)
		{
		file.open(transformFilePath);
		}
	else
		{
		file.open(transformFilePath, std::ios::app);
		}

	ASSERT(file.good(), "Could not write in output transform file");
	file << GetXPosition(*outputCameraPose) << " " << GetYPosition(*outputCameraPose) << " " << GetZPosition(*outputCameraPose) << " " << 
		GetXOrientation(*outputCameraPose) << " " << GetYOrientation(*outputCameraPose) << " " << GetZOrientation(*outputCameraPose) << " " << GetWOrientation(*outputCameraPose) << std::endl;
	file.close();

	index++;
	}

/** @} */
