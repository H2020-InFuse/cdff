/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudModelLocalisation.cpp
 * @date 22/01/2019
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
#include "PointCloudModelLocalisation.hpp"

using namespace CDFF::DFN;
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
PointCloudModelLocalisationTestInterface::PointCloudModelLocalisationTestInterface(const std::string& folderPath, const std::string& baseConfigurationFileName, const std::string& performanceMeasuresFileName,
	CDFF::DFPC::PointCloudModelLocalisationInterface* localizer) : PerformanceTestInterface(folderPath, baseConfigurationFileName, performanceMeasuresFileName)
	{
	this->localizer = localizer;
	SetDfpc(localizer);

	sceneCloud = NULL;
	modelCloud = NULL;
	}

PointCloudModelLocalisationTestInterface::~PointCloudModelLocalisationTestInterface()
	{
	ClearInputs();

	delete(localizer);
	}

void PointCloudModelLocalisationTestInterface::SetInputFilesPath(const std::string& baseFolderPath, const std::string& inputListFileName)
	{
	this->baseFolderPath = baseFolderPath;
	this->inputListFileName = inputListFileName;

	ReadInputFileList();
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */	
PointCloudConstPtr PointCloudModelLocalisationTestInterface::LoadPointCloud(const std::string& cloudFilePath)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile(cloudFilePath, *newCloud);

	return pointCloudConverter.Convert(newCloud);
	}

void PointCloudModelLocalisationTestInterface::LoadGroundTruthPose(const std::string& poseFilePath)
	{
	std::ifstream containerFile(poseFilePath.c_str());
	ASSERT(containerFile.good(), "Error opening pose file");

	std::string line;
	bool lineWasRead = static_cast<bool>(std::getline(containerFile, line));
	ASSERT(lineWasRead, "Error reading pose file, bad line");
	containerFile.close();

	std::vector<std::string> stringsList;
	boost::split(stringsList, line, boost::is_any_of(" "));
	ASSERT(stringsList.size() == 7, "Error reading file, line should contain 7 components of the pose x, y, z, qx, qy, qz, qw");

	try 
		{
		float x = std::stof( stringsList.at(0) );
		float y = std::stof( stringsList.at(1) );
		float z = std::stof( stringsList.at(2) );
		float qx = std::stof( stringsList.at(3) );
		float qy = std::stof( stringsList.at(4) );
		float qz = std::stof( stringsList.at(5) );
		float qw = std::stof( stringsList.at(6) );

		SetPosition(groundTruthPose, x, y, z);
		SetOrientation(groundTruthPose, qx, qy, qz, qw);
		}
	catch ( ... )
		{
		ASSERT(false, "Error parsing pose");
		}	
	}

void PointCloudModelLocalisationTestInterface::ReadInputFileList()
	{
	inputFileList.clear();

	std::stringstream imagesListFilePath;
	imagesListFilePath << baseFolderPath << "/" << inputListFileName;
	std::ifstream containerFile(imagesListFilePath.str().c_str());

	std::string line;
	while (std::getline(containerFile, line))
		{
		std::vector<std::string> stringsList;
		boost::split(stringsList, line, boost::is_any_of(" "));
		ASSERT(stringsList.size() == 3, "Error reading file, bad line");
		
		InputFilePathCollection newEntry;
		newEntry.sceneCloudFilePath = std::string(stringsList.at(0));
		newEntry.modelCloudFilePath = std::string(stringsList.at(1));
		newEntry.groundTruthPoseFilePath = std::string(stringsList.at(2));
		inputFileList.push_back( newEntry );
		}

	containerFile.close();
	}

void PointCloudModelLocalisationTestInterface::ClearInputs()
	{
	if (sceneCloud != NULL)
		{
		delete(sceneCloud);
		sceneCloud = NULL;
		}
	if (modelCloud != NULL)
		{
		delete(modelCloud);
		modelCloud = NULL;
		}	
	}

bool PointCloudModelLocalisationTestInterface::SetNextInputs()
	{
	static unsigned time = 0;
	ClearInputs();

	if (time < inputFileList.size())
		{
		sceneCloud = LoadPointCloud( inputFileList.at(time).sceneCloudFilePath );
		modelCloud = LoadPointCloud( inputFileList.at(time).modelCloudFilePath );
		LoadGroundTruthPose( inputFileList.at(time).groundTruthPoseFilePath );

		time++;
		return true;
		}
	
	return false;
	}

PointCloudModelLocalisationTestInterface::MeasuresMap PointCloudModelLocalisationTestInterface::ExtractMeasures()
	{
	MeasuresMap measuresMap;

	measuresMap["LocationError"] = ComputeLocationError();
	measuresMap["OrientationError"] = ComputeOrientationError(1);

	return measuresMap;
	}

void PointCloudModelLocalisationTestInterface::ExecuteDfpc()
	{
	localizer->sceneInput( *sceneCloud );
	localizer->modelInput( *modelCloud );
	localizer->run();
		
	Copy( localizer->poseOutput(), outputPose);
	success = localizer->successOutput();
	}

float PointCloudModelLocalisationTestInterface::ComputeLocationError()
	{
	float distanceX = GetXPosition(outputPose) - GetXPosition(groundTruthPose);
	float distanceY = GetYPosition(outputPose) - GetYPosition(groundTruthPose);
	float distanceZ = GetZPosition(outputPose) - GetZPosition(groundTruthPose);
	float squaredDistance = distanceX*distanceX + distanceY*distanceY + distanceZ*distanceZ;

	return std::sqrt(squaredDistance);
	}

float PointCloudModelLocalisationTestInterface::ComputeOrientationError(float modelSize)
	{
	Eigen::Quaternion<float> outputRotation
		(
		GetWOrientation(outputPose),
		GetXOrientation(outputPose),
		GetYOrientation(outputPose),
		GetZOrientation(outputPose)
		);

	Eigen::Quaternion<float> truthRotation
		(
		GetWOrientation(groundTruthPose),
		GetXOrientation(groundTruthPose),
		GetYOrientation(groundTruthPose),
		GetZOrientation(groundTruthPose)
		);

	//A point is taken on a sphere of radius modelSize/2; two points are create by rotation this test point by the outputRotation and the truth rotation.
	Eigen::Vector3f testPoint(modelSize/2, 0, 0);
	Eigen::Vector3f outputRotatedPoint = outputRotation * testPoint;
	Eigen::Vector3f truthRotatedPoint = truthRotation * testPoint;

	//The distance between the two points is taken as a measure of the orientation error.
	float differenceX = outputRotatedPoint.x() - truthRotatedPoint.x();
	float differenceY = outputRotatedPoint.y() - truthRotatedPoint.y();
	float differenceZ = outputRotatedPoint.z() - truthRotatedPoint.z();
	float squaredDistance = differenceX*differenceX + differenceY*differenceY + differenceZ*differenceZ;

	return std::sqrt(squaredDistance);
	}

/** @} */
