/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionFromStereo.cpp
 * @date 23/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the DFPC Reconstruction3D with implementation ReconstructionFromStereo.
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
#include <Reconstruction3D/ReconstructionFromStereo.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <MatToFrameConverter.hpp>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>

#include <Converters/SupportTypes.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <Mocks/Common/Converters/Transform3DToMatConverter.hpp>
#include <Mocks/Common/Converters/PclPointCloudToPointCloudConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.hpp>
#include <Mocks/Common/Converters/PointCloudToPclNormalsCloudConverter.hpp>
#include <Mocks/Common/Converters/EigenTransformToTransform3DConverter.cpp>
#include <Stubs/DFPCs/Reconstruction3D/ObservedScene.hpp>

#include <PerformanceTests/DFPCs/PerformanceTestInterface.hpp>

using namespace dfpc_ci;
using namespace FrameWrapper;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;
using namespace Converters::SupportTypes;


class ReconstructionFromStereoTestInterface : public PerformanceTestInterface
	{
	public:
		ReconstructionFromStereoTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName);
		~ReconstructionFromStereoTestInterface();
	protected:

	private:
		Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubFrameCache;
		Mocks::FrameToMatConverter* mockFrameConverter;
		Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubInverseFrameCache;
		Mocks::MatToFrameConverter* mockInverseFrameConverter;
		Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>* stubMatToVectorCache;
		Mocks::MatToVisualPointFeatureVector2DConverter* mockMatToVectorConverter;
		Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>* stubVectorToMatCache;
		Mocks::VisualPointFeatureVector2DToMatConverter* mockVectorToMatConverter;
		Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubEssentialPoseCache;
		Mocks::MatToPose3DConverter* mockEssentialPoseConverter;
		Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>* stubTriangulationPoseCache;
		Mocks::Pose3DToMatConverter* mockTriangulationPoseConverter;
		Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>* stubCloudCache;
		Mocks::PclPointCloudToPointCloudConverter* mockCloudConverter;
		Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>* stubInverseCloudCache;
		Mocks::PointCloudToPclPointCloudConverter* mockInverseCloudConverter;
		Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>* stubOutputCache;
		Mocks::EigenTransformToTransform3DConverter* mockOutputConverter;

		ObservedScene* map;
		ReconstructionFromStereo* reconstructionFromStereo;
		void SetupMocksAndStubs();
	
		std::vector<FrameConstPtr> leftImagesList;
		std::vector<FrameConstPtr> rightImagesList;
		PointCloudConstPtr pointCloud;
		Pose3DConstPtr pose;
		bool success;

		void ClearInputs();
		bool SetNextInputs();
		void ExecuteDfpc();
		MeasuresMap ExtractMeasures();
	};

ReconstructionFromStereoTestInterface::ReconstructionFromStereoTestInterface(std::string folderPath, std::string baseConfigurationFileName, std::string performanceMeasuresFileName)
	: PerformanceTestInterface(folderPath, baseConfigurationFileName, performanceMeasuresFileName)
	{
	map = new ObservedScene();
	reconstructionFromStereo = new ReconstructionFromStereo(map);
	SetDfpc(reconstructionFromStereo);
	SetupMocksAndStubs();

	pointCloud = NULL;
	pose = NULL;
	}

ReconstructionFromStereoTestInterface::~ReconstructionFromStereoTestInterface()
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

	delete(reconstructionFromStereo);
	delete(map);
	}

void ReconstructionFromStereoTestInterface::SetupMocksAndStubs()
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

void ReconstructionFromStereoTestInterface::ClearInputs()
	{
	for(unsigned imageIndex = 0; imageIndex < leftImagesList.size(); imageIndex++)
		{
		delete( leftImagesList.at(imageIndex) );
		delete( rightImagesList.at(imageIndex) );
		}
	leftImagesList.clear();
	rightImagesList.clear();	
	}

bool ReconstructionFromStereoTestInterface::SetNextInputs()
	{
	static unsigned time = 0;
	ClearInputs();

	if (time == 0)
		{
		unsigned numberOfImages = 32;
		for(unsigned imageIndex = 1; imageIndex <= numberOfImages; imageIndex++)
			{
			std::stringstream imageFileStream;
			imageFileStream << "../tests/Data/Images/Scene" << imageIndex << ".png";
			cv::Mat doubleImage = cv::imread(imageFileStream.str(), cv::IMREAD_COLOR);
			cv::Mat cvLeftImage = doubleImage( cv::Rect(0,0,doubleImage.cols/2, doubleImage.rows) );
			cv::Mat cvRightImage = doubleImage( cv::Rect(doubleImage.cols/2,0,doubleImage.cols/2, doubleImage.rows) );

			MatToFrameConverter converter;
			leftImagesList.push_back( converter.Convert(cvLeftImage) );
			rightImagesList.push_back( converter.Convert(cvRightImage) );
			}

		time++;
		return true;
		}
	
	return false;
	}

void ReconstructionFromStereoTestInterface::ExecuteDfpc()
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
		reconstructionFromStereo->leftImageInput( leftImagesList.at(imageIndex) );
		reconstructionFromStereo->rightImageInput( rightImagesList.at(imageIndex) );
		reconstructionFromStereo->run();
		
		pointCloud = reconstructionFromStereo->pointCloudOutput();
		pose = reconstructionFromStereo->poseOutput();
		success = reconstructionFromStereo->successOutput();
		}
	}

ReconstructionFromStereoTestInterface::MeasuresMap ReconstructionFromStereoTestInterface::ExtractMeasures()
	{
	static unsigned testId = 0;
	testId++;
	MeasuresMap measuresMap;


	return measuresMap;
	}


int main(int argc, char** argv)
	{
	ReconstructionFromStereoTestInterface interface("../tests/ConfigurationFiles/DFPCs/Reconstruction3D", "ReconstructionFromStereo_Performance1.yaml", "ReconstructonFromStereoOutput.txt");
	interface.Run();
	};

/** @} */
