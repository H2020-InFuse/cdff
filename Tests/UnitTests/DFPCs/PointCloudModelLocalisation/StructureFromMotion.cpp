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

#include <MatToFrameConverter.hpp>
#include <Frame.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Pose.hpp>

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

#include <Stubs/DFPCs/PointCloudModelLocalisation/ObservedScene.hpp>

using namespace dfpc_ci;
using namespace FrameWrapper;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;


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
	structureFromMotion.setConfigurationFile("../../tests/ConfigurationFiles/DFPCs/PointCloudModelLocalisation/DfpcStructureFromMotion_conf01.yaml");
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

	Map* map = new ObservedScene();
	StructureFromMotion structureFromMotion(map);
	structureFromMotion.setConfigurationFile("../../tests/ConfigurationFiles/DFPCs/PointCloudModelLocalisation/DfpcStructureFromMotion_conf01.yaml");
	structureFromMotion.configure();

	cv::Mat doubleImage1 = cv::imread("../../tests/Data/Images/SmestechLab.jpg", cv::IMREAD_COLOR);
	cv::Mat doubleImage2 = cv::imread("../../tests/Data/Images/SmestechLab.jpg", cv::IMREAD_COLOR);
	
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
	REQUIRE( structureFromMotion.successOutput() == true);
	}


/** @} */
