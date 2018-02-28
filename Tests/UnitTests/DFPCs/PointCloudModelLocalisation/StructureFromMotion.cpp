/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StructureFromMotion.cpp
 * @date 26/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFPCs StructureFromMotion.
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Definitions
 * Catch definition must be before the includes, otherwise catch will not compile.
 *
 * --------------------------------------------------------------------------
 */
#define CATCH_CONFIG_MAIN

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Catch/catch.h>
#include <PointCloudModelLocalisation/StructureFromMotion.hpp>
#include <Errors/Assert.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include<pcl/io/ply_io.h>

#include <MatToFrameConverter.hpp>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Pose.hpp>

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
#include <Mocks/Common/Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>

#include <Stubs/DFPCs/PointCloudModelLocalisation/ObservedScene.hpp>

using namespace dfpc_ci;
using namespace FrameWrapper;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;
using namespace Converters::SupportTypes;


/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Success Call to Configure", "[configureSuccess]" ) 
	{
	Map* map = new ObservedScene();
	StructureFromMotion structureFromMotion(map);
	structureFromMotion.setConfigurationFile("../tests/ConfigurationFiles/DFPCs/PointCloudModelLocalisation/DfpcStructureFromMotion_conf01.yaml");
	structureFromMotion.configure();
	}

TEST_CASE( "Success Call to Process", "[processSuccess]" ) 
	{
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

	Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>* stubFeatures3dCache = new Stubs::CacheHandler<cv::Mat, VisualPointFeatureVector3DConstPtr>();
	Mocks::MatToVisualPointFeatureVector3DConverter* mockFeatures3dConverter = new Mocks::MatToVisualPointFeatureVector3DConverter();
	ConversionCache<cv::Mat, VisualPointFeatureVector3DConstPtr, MatToVisualPointFeatureVector3DConverter>::Instance(stubFeatures3dCache, mockFeatures3dConverter);

	Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>* stubInputNormalsCache = new Stubs::CacheHandler<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr>();
	Mocks::PointCloudToPclNormalsCloudConverter* mockInputNormalsConverter = new Mocks::PointCloudToPclNormalsCloudConverter();
	ConversionCache<PointCloudConstPtr, pcl::PointCloud<pcl::Normal>::ConstPtr, PointCloudToPclNormalsCloudConverter>::Instance(stubInputNormalsCache, mockInputNormalsConverter);

	Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>* stubInputCache = new Stubs::CacheHandler<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures>();
	Mocks::VisualPointFeatureVector3DToPclPointCloudConverter* mockInputConverter = new Mocks::VisualPointFeatureVector3DToPclPointCloudConverter();
	ConversionCache<VisualPointFeatureVector3DConstPtr, PointCloudWithFeatures, VisualPointFeatureVector3DToPclPointCloudConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>* stubOutputCache = new Stubs::CacheHandler<Eigen::Matrix4f, Transform3DConstPtr>();
	Mocks::EigenTransformToTransform3DConverter* mockOutputConverter = new Mocks::EigenTransformToTransform3DConverter();
	ConversionCache<Eigen::Matrix4f, Transform3DConstPtr, EigenTransformToTransform3DConverter>::Instance(stubOutputCache, mockOutputConverter);


	Map* map = new ObservedScene();
	StructureFromMotion structureFromMotion(map);
	structureFromMotion.setConfigurationFile("../tests/ConfigurationFiles/DFPCs/PointCloudModelLocalisation/DfpcStructureFromMotion_conf01.yaml");
	structureFromMotion.configure();

	pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pclModelCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::io::loadPLYFile("../tests/Data/PointClouds/bunny0.ply", *pclCloud);
	const unsigned SELECTION_RATIO = 10;
	unsigned selectionCounter = 0;
	for(unsigned pointIndex = 0; pointIndex < pclCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = pclCloud->points.at(pointIndex);
		if (point.x == point.x && point.y == point.y && point.z == point.z)
			{
			if (selectionCounter == 0)
				{
				pclModelCloud->points.push_back(point);
				}
			selectionCounter = (selectionCounter+1)%SELECTION_RATIO;
			}
		}
	PclPointCloudToPointCloudConverter pclConverter;
	PointCloudConstPtr modelCloud = pclConverter.Convert(pclModelCloud);
	structureFromMotion.modelInput(modelCloud);

	cv::Mat doubleImage1 = cv::imread("../tests/Data/Images/SmestechLabA.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImage2 = cv::imread("../tests/Data/Images/SmestechLabB.jpg", cv::IMREAD_COLOR);
	
	cv::Mat firstCvImage = doubleImage1( cv::Rect(0,0,doubleImage1.cols/2,doubleImage1.rows) );
	cv::Mat secondCvImage = doubleImage2( cv::Rect(0,0,doubleImage1.cols/2,doubleImage1.rows) );

	MatToFrameConverter converter;
	FrameConstPtr firstImage = converter.Convert(firstCvImage);
	FrameConstPtr secondImage = converter.Convert(secondCvImage);
	
	structureFromMotion.imageInput(firstImage);
	structureFromMotion.process();
	REQUIRE( structureFromMotion.successOutput() == false);

	structureFromMotion.imageInput(secondImage);
	structureFromMotion.process();
	//REQUIRE( structureFromMotion.successOutput() == false);
	}


/** @} */
