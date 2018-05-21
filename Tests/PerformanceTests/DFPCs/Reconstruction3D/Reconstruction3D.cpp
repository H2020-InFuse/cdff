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

using namespace dfn_ci;
using namespace Common;
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
	dfpc_ci::Reconstruction3DInterface* reconstructor) : PerformanceTestInterface(folderPath, baseConfigurationFileName, performanceMeasuresFileName)
	{
	this->reconstructor = reconstructor;
	SetDfpc(reconstructor);
	SetupMocksAndStubs();

	pointCloud = NULL;
	pose = NULL;

	saveOutputCloud = false;
	}

Reconstruction3DTestInterface::~Reconstruction3DTestInterface()
	{
	delete(stubFrameCache);
	delete(mockFrameConverter);

	delete(stubInverseFrameCache);
	delete(mockInverseFrameConverter);

	delete(stubMatToVectorCache);
	delete(mockMatToVectorConverter);

	delete(stubVectorToMatCache);
	delete(mockVectorToMatConverter);

	delete(stubEssentialPoseCache);
	delete(mockEssentialPoseConverter);

	delete(stubTriangulationPoseCache);
	delete(mockTriangulationPoseConverter);

	delete(stubCloudCache);
	delete(mockCloudConverter);

	delete(stubInverseCloudCache);
	delete(mockInverseCloudConverter);

	delete(stubOutputCache);
	delete(mockOutputConverter);

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

void Reconstruction3DTestInterface::SetupMocksAndStubs()
	{
	stubFrameCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	mockFrameConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubFrameCache, mockFrameConverter);

	stubInverseFrameCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	mockInverseFrameConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubInverseFrameCache, mockInverseFrameConverter);

	stubMatToVectorCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	mockMatToVectorConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubMatToVectorCache, mockMatToVectorConverter);

	stubVectorToMatCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	mockVectorToMatConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubVectorToMatCache, mockVectorToMatConverter);

	stubEssentialPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	mockEssentialPoseConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubEssentialPoseCache, mockEssentialPoseConverter);

	stubTriangulationPoseCache = new Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>();
	mockTriangulationPoseConverter = new Mocks::Pose3DToMatConverter();
	ConversionCache<Pose3DConstPtr, cv::Mat, Pose3DToMatConverter>::Instance(stubTriangulationPoseCache, mockTriangulationPoseConverter);

	stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>;
	mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);

	stubInverseCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>;
	mockInverseCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInverseCloudCache, mockInverseCloudConverter);

	stubOutputCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	mockOutputConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubOutputCache, mockOutputConverter);
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

		Converters::PointCloudToPclPointCloudConverter pointCloudConverter;
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
		reconstructor->leftImageInput( leftImagesList.at(imageIndex) );
		reconstructor->rightImageInput( rightImagesList.at(imageIndex) );
		reconstructor->run();
		
		pointCloud = reconstructor->pointCloudOutput();
		pose = reconstructor->poseOutput();
		success = reconstructor->successOutput();
		}
	}

/** @} */
