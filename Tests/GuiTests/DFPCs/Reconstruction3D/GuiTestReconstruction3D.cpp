/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file GuiTestReconstruction3D.cpp
 * @date 28/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup GuiTests
 * 
 * Implementation of the GuiTestReconstruction3D class.
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
#include "GuiTestReconstruction3D.hpp"
#include <boost/algorithm/string.hpp>

using namespace dfpc_ci;
using namespace FrameWrapper;
using namespace Converters;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
GuiTestReconstruction3D::GuiTestReconstruction3D(std::string configurationFilePath, std::string imageFilesFolder, std::string imagesListFileName, ImageFilesType imageFilesType) 
	{
	this->configurationFilePath = configurationFilePath;
	this->imageFilesFolder = imageFilesFolder;
	this->imageFilesType = imageFilesType;

	SetUpMocksAndStubs();
	LoadImagesList(imagesListFileName);
	}

GuiTestReconstruction3D::~GuiTestReconstruction3D()
	{
	delete(stubFrameCache);
	delete(mockFrameConverter);

	delete(stubInverseFrameCache);
	delete(mockInverseFrameConverter);

	delete(stubCloudCache);
	delete(mockCloudConverter);

	delete(stubInverseCloudCache);
	delete(mockInverseCloudConverter);
	}

void GuiTestReconstruction3D::Run(dfpc_ci::Reconstruction3DInterface& reconstructor3d)
	{
	reconstructor3d.setConfigurationFile(configurationFilePath);
	reconstructor3d.setup();

	FrameConstPtr leftImage, rightImage;
	while( LoadNextImages(leftImage, rightImage) )
		{
		PRINT_TO_LOG("run", "");
		reconstructor3d.leftImageInput(leftImage);
		reconstructor3d.rightImageInput(rightImage);
		reconstructor3d.run();
		PRINT_TO_LOG("run", "after");
		PointCloudConstPtr pointCloud = reconstructor3d.pointCloudOutput();
		Pose3DConstPtr pose = reconstructor3d.poseOutput();
		bool success = reconstructor3d.successOutput();
		PRINT_TO_LOG("run", "load");
		delete(leftImage);
		delete(rightImage);
		if (pointCloud != NULL)
			{
			delete(pointCloud);
			}
		if (pose != NULL)
			{
			delete(pose);
			}
		}
	}

/* --------------------------------------------------------------------------
 *
 * Private Member Functions
 *
 * --------------------------------------------------------------------------
 */
void GuiTestReconstruction3D::SetUpMocksAndStubs()
	{
	stubFrameCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockFrameConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubFrameCache, mockFrameConverter);

	stubInverseFrameCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	mockInverseFrameConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubInverseFrameCache, mockInverseFrameConverter);

	stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>;
	mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);

	stubInverseCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>;
	mockInverseCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInverseCloudCache, mockInverseCloudConverter);
	}

void GuiTestReconstruction3D::LoadImagesList(std::string imagesListFileName)
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

bool GuiTestReconstruction3D::LoadNextImages(FrameConstPtr& leftImage, FrameConstPtr& rightImage)
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

/** @} */
