/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file IterativePnpSolver.cpp
 * @date 20/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing application for the DFN IterativePnpSolver.
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
#include <catch.hpp>
#include <PerspectiveNPointSolving/IterativePnpSolver.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
#include <ConversionCache/ConversionCache.hpp>
#include <VisualPointFeatureVector2DToMatConverter.hpp>
#include <MatToTransform3DConverter.hpp>
#include <Mocks/Common/Converters/VisualPointFeatureVector2DToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToTransform3DConverter.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
using namespace Common;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (iterative PnP solver)", "[process]" ) 
	{
	Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>* stubInputCache = new Stubs::CacheHandler<VisualPointFeatureVector2DConstPtr, cv::Mat>();
	Mocks::VisualPointFeatureVector2DToMatConverter* mockInputConverter = new Mocks::VisualPointFeatureVector2DToMatConverter();
	ConversionCache<VisualPointFeatureVector2DConstPtr, cv::Mat, VisualPointFeatureVector2DToMatConverter>::Instance(stubInputCache, mockInputConverter);

	Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>* stubOutputCache = new Stubs::CacheHandler<cv::Mat, Pose3DConstPtr>();
	Mocks::MatToPose3DConverter* mockOutputConverter = new Mocks::MatToPose3DConverter();
	ConversionCache<cv::Mat, Pose3DConstPtr, MatToPose3DConverter>::Instance(stubOutputCache, mockOutputConverter);

	const unsigned NUMBER_OF_POINTS = 8;
	cv::Mat inputFeaturesSet(NUMBER_OF_POINTS, 2, CV_32FC1);
	PointCloudPtr pointCloud = new PointCloud();
	ClearPoints(*pointCloud);
	for(unsigned pointIndex = 0; pointIndex < NUMBER_OF_POINTS; pointIndex++)
		{
		inputFeaturesSet.at<float>(pointIndex, 0) = pointIndex * 0.1;
		inputFeaturesSet.at<float>(pointIndex, 1) = pointIndex * 0.1;
		AddPoint(*pointCloud, pointIndex * 0.1 + 0.1, pointIndex * 0.1, 3);	
		}
	mockInputConverter->AddBehaviour("Convert", "1", (void*) (&inputFeaturesSet) );

	VisualPointFeatureVector2DPtr featuresVector = new VisualPointFeatureVector2D();

	Pose3DPtr plannedOutputPose = new Pose3D();
	SetPosition(*plannedOutputPose, 0, 1, 2);
	SetOrientation(*plannedOutputPose, 0, 0, 0, 1);
	mockOutputConverter->AddBehaviour("Convert", "1", (void*) (&plannedOutputPose) );

	IterativePnpSolver pnp;
	pnp.pointCloudInput(pointCloud);
	pnp.cameraFeaturesVectorInput(featuresVector);
	pnp.process();

	Pose3DConstPtr pose = pnp.poseOutput();
	bool success = pnp.successOutput();
	
	REQUIRE( success == true );
	REQUIRE( GetXTranslation(*pose) == GetXTranslation(*plannedOutputPose) );
	REQUIRE( GetYTranslation(*pose) == GetYTranslation(*plannedOutputPose) );
	REQUIRE( GetZTranslation(*pose) == GetZTranslation(*plannedOutputPose) );
	REQUIRE( GetXOrientation(*pose) == GetXOrientation(*plannedOutputPose) );
	REQUIRE( GetYOrientation(*pose) == GetYOrientation(*plannedOutputPose) );
	REQUIRE( GetZOrientation(*pose) == GetZOrientation(*plannedOutputPose) );
	REQUIRE( GetWOrientation(*pose) == GetWOrientation(*plannedOutputPose) );

	delete(pointCloud);
	delete(featuresVector);
	delete(pose);
	}

TEST_CASE( "Call to configure (iterative PnP solver)", "[configure]" )
	{
	IterativePnpSolver pnp;
	pnp.setConfigurationFile("../tests/ConfigurationFiles/DFNs/PerspectiveNPointSolving/IterativePnpSolver_Conf1.yaml");
	pnp.configure();	
	}

/** @} */
