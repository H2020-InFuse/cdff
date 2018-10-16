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
	}

PointCloudGenerator::~PointCloudGenerator()
	{
	delete(disparityMappingExecutor);
	delete(disparityMapping);
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

	SavePointCloud(pointCloud, inputEntry.pose);

	delete(leftImage);
	delete(rightImage);
	}

FrameConstPtr PointCloudGenerator::LoadImage(std::string imageFilePath)
	{
	cv::Mat cvImage = cv::imread(imageFilePath, CV_LOAD_IMAGE_COLOR);
	ASSERT(cvImage.cols > 0 && cvImage.rows >0, "Error: Loaded input image is empty");

	return matFrameConverter.Convert(cvImage);
	}

void PointCloudGenerator::SavePointCloud(PointCloudConstPtr pointCloud, const Pose3D& pose)
	{
	static unsigned saveIndex = 0;

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr pclOutputCloud = pclPointCloudConverter.Convert(pointCloud);
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
