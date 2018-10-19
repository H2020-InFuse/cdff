/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudGenerator.cpp
 * @date 16/10/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Implementation of the PointCloudGenerator class.
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
#include "PointCloudGenerator.hpp"
#include <pcl/io/ply_io.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Converters/PclPointCloudToPointCloudConverter.hpp>


using namespace FrameWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;
using namespace CDFF::DFN::StereoReconstruction;

namespace DataGenerators
{

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PointCloudGenerator::PointCloudGenerator(std::string inputFolderPath, std::string imageFileName, std::string poseFileName, std::string outputFolderPath, std::string cloudFileName, std::string dfnConf)
	{
	this->inputFolderPath = inputFolderPath;
	this->imageFileName = imageFileName;
	this->poseFileName = poseFileName;
	this->outputFolderPath = outputFolderPath;
	this->cloudFileName = cloudFileName;

	disparityMapping = new HirschmullerDisparityMapping;
	disparityMapping->setConfigurationFile(dfnConf);
	disparityMapping->configure();

	disparityMappingExecutor = new CDFF::DFN::StereoReconstructionExecutor(disparityMapping);
	planeFilteringEnabled = false;
	}

PointCloudGenerator::~PointCloudGenerator()
	{
	delete(disparityMappingExecutor);
	delete(disparityMapping);
	}

void PointCloudGenerator::EnablePlaneFiltering()
	{
	planeFilteringEnabled = true;
	}

void PointCloudGenerator::GenerateClouds()
	{
	ReadInputFiles();
	for(unsigned entryIndex = 0; entryIndex < inputList.size(); entryIndex++)
		{
		ExecuteDisparityMapping( inputList.at(entryIndex) );
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void PointCloudGenerator::ReadInputFiles()
	{
	std::string imageFilePath = inputFolderPath + "/" + imageFileName;
	std::string poseFilePath = inputFolderPath + "/" + poseFileName;

	std::ifstream imageFile(imageFilePath);
	std::ifstream poseFile(poseFilePath);
	for(int emptyLineIndex = 0; emptyLineIndex < 3; emptyLineIndex++)
		{
		std::string line;
		std::getline(imageFile, line);
		std::getline(poseFile, line);
		}

	while(imageFile.good() && poseFile.good())
		{
		std::string time;
		std::string leftImageFileName, rightImageFileName;
		float x, y, z, qx, qy, qz, qw;
		imageFile >> time;
		imageFile >> leftImageFileName;
		imageFile >> rightImageFileName;
		poseFile >> time;
		poseFile >> x;
		poseFile >> y;
		poseFile >> z;
		poseFile >> qx;
		poseFile >> qy;
		poseFile >> qz;
		poseFile >> qw;
		
		if (!imageFile.good() || !poseFile.good())
			{
			break;
			}
		
		InputEntry newEntry;
		newEntry.leftImagePath = inputFolderPath + "/" + leftImageFileName;
		newEntry.rightImagePath = inputFolderPath + "/" + rightImageFileName;
		SetPosition(newEntry.pose, x, y, z);
		SetOrientation(newEntry.pose, qx, qy, qz, qw);
		inputList.push_back(newEntry);		
		}
	}

void PointCloudGenerator::ExecuteDisparityMapping(const InputEntry& inputEntry)
	{
	FrameConstPtr leftImage = LoadImage(inputEntry.leftImagePath);
	FrameConstPtr rightImage = LoadImage(inputEntry.rightImagePath);

	PointCloudConstPtr pointCloud = NULL;
	disparityMappingExecutor->Execute(leftImage, rightImage, pointCloud);

	if (planeFilteringEnabled)
		{
		pcl::PointCloud<pcl::PointXYZ>::ConstPtr outputCloud = FilterDominantPlane(pointCloud);
		SavePointCloud(outputCloud, inputEntry.pose);
		}
	else
		{
		SavePointCloud(pointCloud, inputEntry.pose);
		}


	delete(leftImage);
	delete(rightImage);
	}

FrameConstPtr PointCloudGenerator::LoadImage(std::string imageFilePath)
	{
	cv::Mat cvImage = cv::imread(imageFilePath, CV_LOAD_IMAGE_COLOR);
	ASSERT(cvImage.cols > 0 && cvImage.rows >0, "Error: Loaded input image is empty");

	return matFrameConverter.Convert(cvImage);
	}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr PointCloudGenerator::FilterDominantPlane(PointCloudConstPtr pointCloud)
	{
	Converters::PointCloudToPclPointCloudConverter cloudConverter;
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = cloudConverter.Convert(pointCloud);
	
	pcl::SACSegmentation<pcl::PointXYZ> ransacSegmentation;
	ransacSegmentation.setOptimizeCoefficients (true);
	ransacSegmentation.setModelType (pcl::SACMODEL_PLANE);
	ransacSegmentation.setMethodType (pcl::SAC_RANSAC);
	ransacSegmentation.setDistanceThreshold (0.05);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	ransacSegmentation.setInputCloud (cloud);
	ransacSegmentation.segment (*inliers, *coefficients);

	pcl::PointCloud<pcl::PointXYZ>::Ptr leftoverCloud( new pcl::PointCloud<pcl::PointXYZ>() );
	int leftoverIndex = 0;	
	for(int indexIndex = 0; indexIndex < inliers->indices.size(); indexIndex++)
		{
		int pointIndex = inliers->indices.at(indexIndex);
		while (leftoverIndex < pointIndex)
			{
			pcl::PointXYZ leftoverPoint= cloud->points.at(leftoverIndex);
			leftoverCloud->points.push_back(leftoverPoint);
			leftoverIndex++;
			}
		leftoverIndex++;
		}

	return leftoverCloud;
	}

void PointCloudGenerator::SavePointCloud(PointCloudConstPtr pointCloud, const Pose3D& pose)
	{
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclOutputCloud = pclPointCloudConverter.Convert(pointCloud);
	SavePointCloud(pclOutputCloud, pose);
	}

void PointCloudGenerator::SavePointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclOutputCloud, const PoseWrapper::Pose3D& pose)
	{
	static unsigned saveIndex = 0;

	std::string plyFileName = "cloud_" + std::to_string(saveIndex) + ".ply";
	std::string plyFilePath = outputFolderPath + "/" + plyFileName;

	pcl::PLYWriter writer;
	writer.write(plyFilePath, *pclOutputCloud, true);

	std::string cloudFilePath = outputFolderPath + "/" + cloudFileName;
	std::ofstream cloudFile;
	if (saveIndex == 0)
		{
		cloudFile.open(cloudFilePath);
		}
	else
		{
		cloudFile.open(cloudFilePath, std::ios::app);
		}

	cloudFile << plyFileName << " " << GetXPosition(pose) << " " << GetYPosition(pose) << " " << GetZPosition(pose) << " " <<
		GetXOrientation(pose) << " " << GetYOrientation(pose) << " " << GetZOrientation(pose) << " " << GetWOrientation(pose) << std::endl;
	cloudFile.close();

	saveIndex++;
	}

}
/** @} */
