/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Reconstruction3D.cpp
 * @date 23/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Implementation of the Test interface for the performance test of DFPC Reconstruction 3D
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
#include "Reconstruction3D.hpp"

using namespace CDFF::DFN::WHICH-DFN(S)-IF-ANY?;
using namespace Converters;
using namespace FrameWrapper;
using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace PoseWrapper;


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Reconstruction3DTestInterface::Reconstruction3DTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName, 
	CDFF::DFPC::Reconstruction3DInterface* reconstructor) : PerformanceTestInterface(folderPath, baseConfigurationFileName, performanceMeasuresFileName)
	{
	this->reconstructor = reconstructor;
	SetDfpc(reconstructor);

	pointCloud = NULL;
	pose = NULL;

	saveOutputCloud = false;
	}

Reconstruction3DTestInterface::~Reconstruction3DTestInterface()
	{
	if (pointCloud != NULL)
		{
		delete(pointCloud);
		}
	if (pose != NULL)
		{
		delete(pose);
		}
	ClearInputs();

	delete(reconstructor);
	delete(map);
	}

void Reconstruction3DTestInterface::SetImageFilesPath(std::string baseFolderPath, std::string imagesListFileName)
	{
	this->baseFolderPath = baseFolderPath;
	this->imagesListFileName = imagesListFileName;
	ReadImagesList();
	}

void Reconstruction3DTestInterface::SetCloudOutputFile(std::string outputCloudFileBaseName)
	{
	saveOutputCloud = true;
	this->outputCloudFileBaseName = outputCloudFileBaseName;
	}


/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void Reconstruction3DTestInterface::ReadImagesList()
	{
	leftImageFileNamesList.clear();
	rightImageFileNamesList.clear();

	std::stringstream imagesListFilePath;
	imagesListFilePath << baseFolderPath << "/" << imagesListFileName;
	std::ifstream containerFile(imagesListFilePath.str().c_str());

	std::string line;
	while (std::getline(containerFile, line))
		{
		std::vector<std::string> stringsList;
		boost::split(stringsList, line, boost::is_any_of(" "));
		ASSERT(stringsList.size() == 2, "Error reading file, bad line");
		
		leftImageFileNamesList.push_back( std::string(stringsList.at(0)) );
		rightImageFileNamesList.push_back( std::string(stringsList.at(1)) );
		}

	containerFile.close();
	}

void Reconstruction3DTestInterface::ClearInputs()
	{
	for(unsigned imageIndex = 0; imageIndex < leftImagesList.size(); imageIndex++)
		{
		delete( leftImagesList.at(imageIndex) );
		delete( rightImagesList.at(imageIndex) );
		}
	leftImagesList.clear();
	rightImagesList.clear();	
	}

bool Reconstruction3DTestInterface::SetNextInputs()
	{
	static unsigned time = 0;
	ClearInputs();

	if (time == 0)
		{
		unsigned numberOfImages = 32;
		for(unsigned imageIndex = 1; imageIndex <= numberOfImages; imageIndex++)
			{
			std::stringstream leftImageFileStream, rightImageFileStream;
			leftImageFileStream << baseFolderPath << "/" << leftImageFileNamesList.at(imageIndex);
			rightImageFileStream << baseFolderPath << "/" << rightImageFileNamesList.at(imageIndex);

			cv::Mat cvLeftImage = cv::imread(leftImageFileStream.str(), cv::IMREAD_COLOR);
			cv::Mat cvRightImage = cv::imread(rightImageFileStream.str(), cv::IMREAD_COLOR);

			MatToFrameConverter converter;
			leftImagesList.push_back( converter.Convert(cvLeftImage) );
			rightImagesList.push_back( converter.Convert(cvRightImage) );
			}

		time++;
		return true;
		}
	
	return false;
	}

Reconstruction3DTestInterface::MeasuresMap Reconstruction3DTestInterface::ExtractMeasures()
	{
	static unsigned testId = 0;
	testId++;
	MeasuresMap measuresMap;

	if (saveOutputCloud && success)
		{
		std::stringstream outputFilePath;
		outputFilePath << outputCloudFileBaseName << testId << ".ply";

		pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclPointCloud = pointCloudConverter.Convert(pointCloud);

		pcl::PLYWriter writer;
		writer.write(outputFilePath.str(), *pclPointCloud, true);		
		}

	return measuresMap;
	}

void Reconstruction3DTestInterface::ExecuteDfpc()
	{
	for(unsigned imageIndex = 0; imageIndex < leftImagesList.size(); imageIndex++)
		{
		if (pointCloud != NULL)
			{
			delete(pointCloud);
			}
		if (pose != NULL)
			{
			delete(pose);
			}
		reconstructor->leftImageInput( *(leftImagesList.at(imageIndex)) );
		reconstructor->rightImageInput( *(rightImagesList.at(imageIndex)) );
		reconstructor->run();
		
		PointCloudPtr newPointCloud = NewPointCloud();
		Pose3DPtr newPose = NewPose3D();
		Copy( reconstructor->pointCloudOutput(), *newPointCloud);
		Copy( reconstructor->poseOutput(), *newPose);
		pointCloud = newPointCloud;
		pose = newPose;
		success = reconstructor->successOutput();
		}
	}

/** @} */
