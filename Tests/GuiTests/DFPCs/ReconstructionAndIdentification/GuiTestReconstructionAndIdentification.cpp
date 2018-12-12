/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file GuiTestReconstructionAndLocalisation.cpp
 * @date 28/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 *
 * Implementation of the GuiTestReconstructionAndLocalisation class.
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
#include "GuiTestReconstructionAndIdentification.hpp"
#include <boost/algorithm/string.hpp>
#include <pcl/io/ply_io.h>

using namespace FrameWrapper;
using namespace Converters;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
GuiTestReconstructionAndLocalisation::GuiTestReconstructionAndLocalisation(const std::string& configurationFilePath, const std::string& imageFilesFolder, const std::string& imagesListFileName, 
	ImageFilesType imageFilesType, const std::string& modelFilePath) :
	configurationFilePath(configurationFilePath),
	imageFilesFolder(imageFilesFolder),
	modelFilePath(modelFilePath)
	{
	this->imageFilesType = imageFilesType;

	LoadImagesList(imagesListFileName);
	}

GuiTestReconstructionAndLocalisation::~GuiTestReconstructionAndLocalisation()
	{

	}

void GuiTestReconstructionAndLocalisation::Run(CDFF::DFPC::ReconstructionAndIdentificationInterface& reconstructorAndIdentifier)
	{
	reconstructorAndIdentifier.setConfigurationFile(configurationFilePath);
	reconstructorAndIdentifier.setup();

	PointCloudConstPtr modelCloud = LoadPointCloud(modelFilePath);
	reconstructorAndIdentifier.modelInput(*modelCloud);
	reconstructorAndIdentifier.computeModelFeaturesInput(false);
	FrameConstPtr leftImage, rightImage;
	while( LoadNextImages(leftImage, rightImage) )
		{
		PRINT_TO_LOG("run", "");
		reconstructorAndIdentifier.leftImageInput(*leftImage);
		reconstructorAndIdentifier.rightImageInput(*rightImage);
		reconstructorAndIdentifier.run();
		PRINT_TO_LOG("run", "after");
		reconstructorAndIdentifier.pointCloudOutput();
		PRINT_TO_LOG("run", "load");
		delete(leftImage);
		delete(rightImage);
		}

	delete(modelCloud);
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void GuiTestReconstructionAndLocalisation::LoadImagesList(const std::string& imagesListFileName)
	{
	std::stringstream imagesListFilePath;
	imagesListFilePath << imageFilesFolder << "/" << imagesListFileName;
	std::ifstream listFile( imagesListFilePath.str().c_str() );

	ASSERT(listFile.good(), "Error, could not open images list file");

	std::string line;

	std::getline(listFile, line);
	std::getline(listFile, line);
	std::getline(listFile, line);

	while (std::getline(listFile, line))
		{
		std::vector<std::string> stringsList;
		boost::split(stringsList, line, boost::is_any_of(" "));
		imageFileNamesList.push_back( stringsList.at(1) );
		PRINT_TO_LOG("", stringsList.at(1));

		if (imageFilesType == STEREO_CAMERA_TWO_FILES)
			{
			secondImageFileNamesList.push_back( stringsList.at(2) );
			}
		}

	listFile.close();
	}

bool GuiTestReconstructionAndLocalisation::LoadNextImages(FrameConstPtr& leftImage, FrameConstPtr& rightImage)
	{
	static unsigned time = 0;
	if (time >= imageFileNamesList.size() )
		{
		return false;
		}

	std::stringstream nextImageFileName;
	nextImageFileName << imageFilesFolder << "/" << imageFileNamesList.at(time);
	cv::Mat cvLeftImage, cvRightImage;

	std::stringstream inputStream;
	inputStream << (time+1) << "/" << imageFileNamesList.size();
	std::string inputString = inputStream.str();
	PRINT_TO_LOG("input", inputString);

	if (imageFilesType == MONO_CAMERA)
		{
		cvLeftImage = cv::imread(nextImageFileName.str(), cv::IMREAD_COLOR);
		cvRightImage = cvLeftImage.clone();
		}
	else if (imageFilesType == STEREO_CAMERA_ONE_FILE)
		{
		cv::Mat cvImage = cv::imread(nextImageFileName.str(), cv::IMREAD_COLOR);
		cvLeftImage = cvImage( cv::Rect(0, 0, cvImage.cols/2, cvImage.rows) );
		cvRightImage = cvImage( cv::Rect(cvImage.cols/2, 0, cvImage.cols/2, cvImage.rows) );
		}
	else if (imageFilesType == STEREO_CAMERA_TWO_FILES)
		{
		std::stringstream secondImageFileName;
		secondImageFileName << imageFilesFolder << "/" << secondImageFileNamesList.at(time);

		cvLeftImage = cv::imread(nextImageFileName.str(), cv::IMREAD_COLOR);
		cvRightImage = cv::imread(secondImageFileName.str(), cv::IMREAD_COLOR);
		}
	else
		{
		ASSERT(false, "Unhandled image file type");
		}

	leftImage = frameConverter.Convert(cvLeftImage);
	rightImage = frameConverter.Convert(cvRightImage);

	time++;
	return true;
	}

PointCloudConstPtr GuiTestReconstructionAndLocalisation::LoadPointCloud(const std::string& file)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>() );
	pcl::io::loadPLYFile(file, *pclCloud);
	PointCloudConstPtr cloud = pointCloudConverter.Convert(pclCloud);
	return cloud;
	}

/** @} */
