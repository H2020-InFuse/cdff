/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ObservedScene.cpp
 * @date 27/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the ObservedScene Stub Class.
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
#include <Stubs/DFPCs/PointCloudModelLocalisation/ObservedScene.hpp>
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
#include <Mocks/Common/Converters/PclPointCloudToPointCloudConverter.hpp>

using namespace dfpc_ci;
using namespace FrameWrapper;
using namespace Converters;
using namespace VisualPointFeatureVector2DWrapper;
using namespace PoseWrapper;
using namespace Common;
using namespace PointCloudWrapper;
using namespace BaseTypesWrapper;


/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

void ConfigureStubsAndMocks()
	{
	static bool executed = false;
	if (executed)
		{
		return;
		}
	executed = true;

	Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>* stubCloudCache = new Stubs::CacheHandler<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr>;
	Mocks::PclPointCloudToPointCloudConverter* mockCloudConverter = new Mocks::PclPointCloudToPointCloudConverter();
	ConversionCache<pcl::PointCloud<pcl::PointXYZ>::ConstPtr, PointCloudConstPtr, PclPointCloudToPointCloudConverter>::Instance(stubCloudCache, mockCloudConverter);
	}

TEST_CASE( "Adding Frame", "[addingFrame]" ) 
	{
	ConfigureStubsAndMocks();

	ObservedScene observedScene;
	FramePtr frame1 = new Frame();
	FramePtr frame2 = new Frame();
	FramePtr frame3 = new Frame();
	FramePtr frame4 = new Frame();

	observedScene.AddFrame(frame1);
	FrameConstPtr reference = observedScene.GetNextReferenceFrame();
	REQUIRE( reference == NULL);

	observedScene.AddFrame(frame2);
	reference = observedScene.GetNextReferenceFrame();
	REQUIRE( reference == frame1 );

	reference = observedScene.GetNextReferenceFrame();
	REQUIRE( reference == NULL);

	PointCloudPtr cloud12 = new PointCloud();
	ClearPoints(*cloud12);
	Pose3DPtr pose12 = new Pose3D();
	SetPosition(*pose12, 0, 0, 0);
	SetOrientation(*pose12, 0, 0, 0, 1);
	observedScene.AddPointCloud(cloud12, pose12);

	observedScene.AddFrame(frame3);
	reference = observedScene.GetNextReferenceFrame();
	REQUIRE( reference == frame2 );	
	reference = observedScene.GetNextReferenceFrame();
	REQUIRE( reference == frame1 );
	reference = observedScene.GetNextReferenceFrame();
	REQUIRE( reference == NULL);

	//No position for Frame3 was provided, so it will be skipped.
	observedScene.AddFrame(frame4);
	reference = observedScene.GetNextReferenceFrame();
	REQUIRE( reference == frame2 );	
	reference = observedScene.GetNextReferenceFrame();
	REQUIRE( reference == frame1 );
	reference = observedScene.GetNextReferenceFrame();
	REQUIRE( reference == NULL);
	}

TEST_CASE( "Adding Point Cloud", "[addingPointCloud]" ) 
	{
	ConfigureStubsAndMocks();
	
	ObservedScene observedScene;
	FramePtr frame1 = new Frame();
	observedScene.AddFrame(frame1);

	FramePtr frame2 = new Frame();
	observedScene.AddFrame(frame2);

	PointCloudPtr cloud12 = new PointCloud();
	ClearPoints(*cloud12);
	Pose3DPtr pose12 = new Pose3D();
	SetPosition(*pose12, 0, 0, 0);
	SetOrientation(*pose12, 0, 0, 0, 1);
	observedScene.AddPointCloud(cloud12, pose12);

	Point3D origin;
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;
	float radius = 10;
	PointCloudConstPtr scene = observedScene.GetPartialScene( origin, radius);
	REQUIRE( GetNumberOfPoints(*scene) == 0 );
	}

TEST_CASE( "Translation Transform", "[translation]")
	{
	ConfigureStubsAndMocks();

	ObservedScene observedScene;
	FramePtr frame1 = new Frame();
	observedScene.AddFrame(frame1);

	FramePtr frame2 = new Frame();
	observedScene.AddFrame(frame2);

	PointCloudPtr cloud12 = new PointCloud();
	ClearPoints(*cloud12);
	AddPoint(*cloud12, 1, 1, 1);
	Pose3DPtr pose12 = new Pose3D();
	SetPosition(*pose12, 1, 0, 0);
	SetOrientation(*pose12, 0, 0, 0, 1);
	observedScene.AddPointCloud(cloud12, pose12);

	Point3D origin;
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;
	float radius = 10;
	PointCloudConstPtr scene = observedScene.GetPartialScene( origin, radius);
	REQUIRE( GetNumberOfPoints(*scene) == 1 );
	REQUIRE( GetXCoordinate(*scene, 0) == 2 );
	REQUIRE( GetYCoordinate(*scene, 0) == 1 );
	REQUIRE( GetZCoordinate(*scene, 0) == 1 );
	}

TEST_CASE( "Rotation Transform", "[rotation]")
	{
	ConfigureStubsAndMocks();

	ObservedScene observedScene;
	FramePtr frame1 = new Frame();
	observedScene.AddFrame(frame1);

	FramePtr frame2 = new Frame();
	observedScene.AddFrame(frame2);

	PointCloudPtr cloud12 = new PointCloud();
	ClearPoints(*cloud12);
	AddPoint(*cloud12, 1, 1, 1);
	Pose3DPtr pose12 = new Pose3D();
	SetPosition(*pose12, 0, 0, 0);
	SetOrientation(*pose12, 0, 1, 0, 0);
	observedScene.AddPointCloud(cloud12, pose12);

	Point3D origin;
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;
	float radius = 10;
	PointCloudConstPtr scene = observedScene.GetPartialScene( origin, radius);
	REQUIRE( GetNumberOfPoints(*scene) == 1 );
	REQUIRE( GetXCoordinate(*scene, 0) == -1 );
	REQUIRE( GetYCoordinate(*scene, 0) == 1 );
	REQUIRE( GetZCoordinate(*scene, 0) == -1 );
	}

TEST_CASE( "RotoTranslation Transform", "[rototranslation]")
	{
	ConfigureStubsAndMocks();

	ObservedScene observedScene;
	FramePtr frame1 = new Frame();
	observedScene.AddFrame(frame1);

	FramePtr frame2 = new Frame();
	observedScene.AddFrame(frame2);

	PointCloudPtr cloud12 = new PointCloud();
	ClearPoints(*cloud12);
	AddPoint(*cloud12, 1, 1, 1);
	Pose3DPtr pose12 = new Pose3D();
	SetPosition(*pose12, 1, 0, 0);
	SetOrientation(*pose12, 0, 1, 0, 0);
	observedScene.AddPointCloud(cloud12, pose12);

	Point3D origin;
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;
	float radius = 10;
	PointCloudConstPtr scene = observedScene.GetPartialScene( origin, radius);
	REQUIRE( GetNumberOfPoints(*scene) == 1 );
	REQUIRE( GetXCoordinate(*scene, 0) == 0 );
	REQUIRE( GetYCoordinate(*scene, 0) == 1 );
	REQUIRE( GetZCoordinate(*scene, 0) == -1 );
	}


/** @} */
