/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file RecomstructionFromMotion.cpp
 * @date 08/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFPCsTest
 * 
 * Testing application for the DFPC ReconstructionFromMotion.
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
#include <Reconstruction3D/ReconstructionFromMotion.hpp>
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
#include <Visualizers/OpencvVisualizer.hpp>
#include <Visualizers/PclVisualizer.hpp>

using namespace dfpc_ci;
using namespace FrameWrapper;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;
using namespace Converters::SupportTypes;

int main(int argc, char** argv)
	{
	Visualizers::OpencvVisualizer::Enable();
	Visualizers::PclVisualizer::Enable();

	Stubs::CacheHandler<FrameConstPtr, cv::Mat>* stubFrameCache = new Stubs::CacheHandler<FrameConstPtr, cv::Mat>();
	Mocks::FrameToMatConverter* mockFrameConverter = new Mocks::FrameToMatConverter();
	ConversionCache<FrameConstPtr, cv::Mat, FrameToMatConverter>::Instance(stubFrameCache, mockFrameConverter);

	Stubs::CacheHandler<cv::Mat, FrameConstPtr>* stubInverseFrameCache = new Stubs::CacheHandler<cv::Mat, FrameConstPtr>();
	Mocks::MatToFrameConverter* mockInverseFrameConverter = new Mocks::MatToFrameConverter();
	ConversionCache<cv::Mat, FrameConstPtr, MatToFrameConverter>::Instance(stubInverseFrameCache, mockInverseFrameConverter);

	Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>* stubMatToVectorCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector2DConstPtr>();
	Mocks::MatToVisualPointFeatureVector2DConverter* mockMatToVectorConverter = new Mocks::MatToVisualPointFeatureVector2DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector2DConstPtr, MatToVisualPointFeatureVector2DConverter>::Instance(stubMatToVectorCache, mockMatToVectorConverter);

	Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>* stubVectorToMatCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	Mocks::VisualPointFeatureVector2DToMatConverter* mockVectorToMatConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubVectorToMatCache, mockVectorToMatConverter);

	Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubEssentialPoseCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	Mocks::MatToPose3DConverter* mockEssentialPoseConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubEssentialPoseCache, mockEssentialPoseConverter);

	Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>* stubTriangulationPoseCache = new Stubs::CacheHandler<Pose3DConstPtr, cv::Mat>();
	Mocks::Pose3DToMatConverter* mockTriangulationPoseConverter = new Mocks::Pose3DToMatConverter();
	ConversionCache<Pose3DConstPtr, cv::Mat, Pose3DToMatConverter>::Instance(stubTriangulationPoseCache, mockTriangulationPoseConverter);

	Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>* stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>;
	Mocks::PclPointCloudToPointCloudConverter* mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);

	Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>* stubInverseCloudCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr>;
	Mocks::PointCloudToPclPointCloudConverter* mockInverseCloudConverter = new Mocks::PointCloudToPclPointCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudToPclPointCloudConverter>::Instance(stubInverseCloudCache, mockInverseCloudConverter);

	Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>* stubOutputCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	Mocks::EigenTransformToTransform3DConverter* mockOutputConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	WRITE_TO_LOG("Configure", "");
	Map* map = new ObservedScene();
	ReconstructionFromMotion reconstructionFromMotion(map);
	reconstructionFromMotion.setConfigurationFile("../../tests/ConfigurationFiles/DFPCs/Reconstruction3D/DfpcReconstructionFromMotion_conf01.yaml");
	reconstructionFromMotion.setup();
	WRITE_TO_LOG("Configure", "Completed");

	MatToFrameConverter converter;
	unsigned numberOfScenes = 32;
	PointCloudConstPtr pointCloud = NULL;
	Pose3DConstPtr pose = NULL;
	for(unsigned sceneIndex = 1; sceneIndex <= numberOfScenes; sceneIndex++)
		{
		if (pointCloud != NULL)
			{
			delete(pointCloud);
			}
		if (pose != NULL)
			{
			delete(pose);
			}

		std::stringstream sceneFileName;
		sceneFileName << "../../tests/Data/Images/Scene"<<sceneIndex<<".png";
		cv::Mat doubleImage = cv::imread(sceneFileName.str(), cv::IMREAD_COLOR);
		cv::Mat cvleftImage = doubleImage( cv::Rect(0,0,doubleImage.cols/2,doubleImage.rows) );	
		cv::Mat cvRightImage = doubleImage( cv::Rect(doubleImage.cols/2,0,doubleImage.cols/2,doubleImage.rows) );
		FrameConstPtr leftImage = converter.Convert(cvleftImage);
		FrameConstPtr rightImage = converter.Convert(cvRightImage);

		WRITE_TO_LOG("Image Input Number", sceneIndex);
		reconstructionFromMotion.leftImageInput(leftImage);
		reconstructionFromMotion.rightImageInput(rightImage);
		reconstructionFromMotion.run();
		WRITE_TO_LOG("Result", reconstructionFromMotion.successOutput() );

		pointCloud = reconstructionFromMotion.pointCloudOutput();
		pose = reconstructionFromMotion.poseOutput();
		}

	PRINT_LOG();
	if (pointCloud == NULL)
		{
		return 0;
		}


	Visualizers::PclVisualizer::Enable();
	DEBUG_SHOW_POINT_CLOUD(pointCloud);
 
	return 0;
	};

/** @} */
