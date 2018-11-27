/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudMap.cpp
 * @date 23/08/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the PointCloudMap Class.
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
#include <Reconstruction3D/PointCloudMap.hpp>
#include <Errors/Assert.hpp>

using namespace VisualPointFeatureVector3DWrapper;
using namespace PointCloudWrapper;
using namespace CDFF::DFPC::Reconstruction3D;
using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

#define EPSILON 0.001
#define CLOSE(a, b) (b > a - EPSILON && b < a + EPSILON)
#define CLOSE_POINT(cloud, index, x, y, z) ( CLOSE(GetXCoordinate(cloud, index), x) && CLOSE(GetYCoordinate(cloud, index), y) && CLOSE(GetZCoordinate(cloud, index), z))
#define CLOSE_POSITION(pose, x, y, z) ( CLOSE(GetXPosition(pose), x) && CLOSE(GetYPosition(pose), y) && CLOSE(GetZPosition(pose), z))
#define CLOSE_HALF_ORIENTATION(pose, x, y, z, w) ( CLOSE(GetXOrientation(pose), x) && CLOSE(GetYOrientation(pose), y) && CLOSE(GetZOrientation(pose), z) && CLOSE(GetWOrientation(pose), w))
#define CLOSE_ORIENTATION(pose, x, y, z, w) ( CLOSE_HALF_ORIENTATION(pose, x, y, z, w) || CLOSE_HALF_ORIENTATION(pose, -x, -y, -z, -w) )

TEST_CASE( "Add and Get (PointCloudMap)", "[AddAndGet]" ) 
	{
	PointCloudMap* map = new PointCloudMap();

	PointCloudPtr cloud1 = NewPointCloud();
	PointCloudPtr cloud2 = NewPointCloud();
	PointCloudPtr cloud3 = NewPointCloud();	

	AddPoint(*cloud1, 1, 1, 1);
	AddPoint(*cloud1, 1.1, 1.1, 1.1);
	AddPoint(*cloud2, 2, 2, 2);
	AddPoint(*cloud2, 2.2, 2.2, 2.2);
	AddPoint(*cloud3, 3, 3, 3);
	AddPoint(*cloud3, 3.3, 3.3, 3.3);

	VisualPointFeatureVector3DPtr vector1 = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr vector2 = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr vector3 = NewVisualPointFeatureVector3D();

	AddPoint(*vector1, 1, 1, 1);
	AddPoint(*vector2, 2, 2, 2);
	AddPoint(*vector3, 3, 3, 3);

	Pose3DPtr origin1 = NewPose3D();
	Pose3DPtr origin2 = NewPose3D();
	Pose3DPtr pose1 = NewPose3D();
	Pose3DPtr pose2 = NewPose3D();
	Pose3DPtr pose3 = NewPose3D();
	
	SetPosition(*origin1, 0, 0, 0);
	SetPosition(*origin2, 1, 1, 1);
	SetPosition(*pose1, 0, 0, 0);
	SetPosition(*pose2, 1, 1, 1);
	SetPosition(*pose3, 1, 1, 1);

	SetOrientation(*origin1, 0, 0, 0, 1);
	SetOrientation(*origin2, 0, 0, 1, 0);
	SetOrientation(*pose1, 0, 0, 0, 1);
	SetOrientation(*pose2, 0, 0, 1, 0);
	SetOrientation(*pose3, 0, 0, 1, 0);

	map->AddPointCloud(cloud1, vector1, pose1);

	const Pose3D& latestPose1 = map->GetLatestPose();
	CLOSE_POSITION(latestPose1, 0, 0, 0);
	CLOSE_ORIENTATION(latestPose1, 0, 0, 0, 1);

	PointCloudConstPtr sceneCloudBase1 = map->GetScenePointCloud(origin1, 100);
	REQUIRE( GetNumberOfPoints(*sceneCloudBase1) == 2 );
	CLOSE_POINT( *sceneCloudBase1, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloudBase1, 1, 1.1, 1.1, 1.1);

	map->AddPointCloud(cloud2, vector2, pose2);

	const Pose3D& latestPose2 = map->GetLatestPose();
	CLOSE_POSITION(latestPose2, 1, 1, 1);
	CLOSE_ORIENTATION(latestPose2, 0, 0, 1, 0);

	PointCloudConstPtr sceneCloudBase2 = map->GetScenePointCloud(origin1, 100);
	REQUIRE( GetNumberOfPoints(*sceneCloudBase2) == 4 );
	CLOSE_POINT( *sceneCloudBase2, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloudBase2, 1, 1.1, 1.1, 1.1);
	CLOSE_POINT( *sceneCloudBase2, 2, -1, -1, 1);
	CLOSE_POINT( *sceneCloudBase2, 3, -1.2, -1.2, 1.2);

	map->AttachPointCloud(cloud3, vector3, pose3);

	const Pose3D& latestPose3 = map->GetLatestPose();
	CLOSE_POSITION(latestPose3, 2, 2, 2);
	CLOSE_ORIENTATION(latestPose3, 0, 0, 0, 1);

	PointCloudConstPtr sceneCloud1 = map->GetScenePointCloud(origin1, 1.74);
	REQUIRE( GetNumberOfPoints(*sceneCloud1) == 2 );
	CLOSE_POINT( *sceneCloud1, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloud1, 1, -1, -1, 1);

	PointCloudConstPtr sceneCloud2 = map->GetScenePointCloud(origin1, 8.00);
	REQUIRE( GetNumberOfPoints(*sceneCloud2) == 5 );
	CLOSE_POINT( *sceneCloud2, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloud2, 1, 1.1, 1.1, 1.1);
	CLOSE_POINT( *sceneCloud2, 2, -1, -1, 1);
	CLOSE_POINT( *sceneCloud2, 3, -1.2, -1.2, 1.2);
	CLOSE_POINT( *sceneCloud2, 4, 1.3, 1.3, 1.3);

	PointCloudConstPtr sceneCloud3 = map->GetScenePointCloudInOrigin(origin1, 1.74);
	REQUIRE( GetNumberOfPoints(*sceneCloud3) == 2 );
	CLOSE_POINT( *sceneCloud3, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloud3, 1, -1, -1, 1);

	PointCloudConstPtr sceneCloud4 = map->GetScenePointCloudInOrigin(origin2, 8.00);
	REQUIRE( GetNumberOfPoints(*sceneCloud4) == 5 );
	CLOSE_POINT( *sceneCloud4, 0, 0, 0, 0);
	CLOSE_POINT( *sceneCloud4, 1, -0.1, -0.1, 0.1);
	CLOSE_POINT( *sceneCloud4, 2, -2, -2, 0);
	CLOSE_POINT( *sceneCloud4, 3, -2.2, -2.2, 0.2);
	CLOSE_POINT( *sceneCloud4, 4, -0.3, -0.3, 0.3);

	VisualPointFeatureVector3DConstPtr featureCloud1 = map->GetSceneFeaturesVector(origin1, 1.74);
	REQUIRE( GetNumberOfPoints(*featureCloud1) == 2 );
	CLOSE_POINT( *featureCloud1, 0, 1, 1, 1);
	CLOSE_POINT( *featureCloud1, 1, -1, -1, 1);

	VisualPointFeatureVector3DConstPtr featureCloud2 = map->GetSceneFeaturesVector(origin1, 6.00);
	REQUIRE( GetNumberOfPoints(*featureCloud2) == 2 );
	CLOSE_POINT( *featureCloud2, 0, 1, 1, 1);
	CLOSE_POINT( *featureCloud2, 1, -1, -1, 1);

	delete(map);
	delete(cloud1);
	delete(cloud2);
	delete(cloud3);
	delete(origin1);
	delete(origin2);
	delete(pose1);
	delete(pose2);
	delete(pose3);
	delete(vector1);
	delete(vector2);
	delete(vector3);

	delete(sceneCloud1);
	delete(sceneCloud2);
	delete(sceneCloud3);
	delete(sceneCloud4);
	delete(featureCloud1);
	delete(featureCloud2);
	}


TEST_CASE( "Add and Get 2 (PointCloudMap)", "[AddAndGet]" ) 
	{
	PointCloudMap* map = new PointCloudMap();

	PointCloudPtr cloud1 = NewPointCloud();
	PointCloudPtr cloud2 = NewPointCloud();
	PointCloudPtr cloud3 = NewPointCloud();	

	AddPoint(*cloud1, 1, 1, 1);
	AddPoint(*cloud1, 1.1, 1.1, 1.1);
	AddPoint(*cloud2, 2, 2, 2);
	AddPoint(*cloud2, 2.2, 2.2, 2.2);
	AddPoint(*cloud3, 3, 3, 3);
	AddPoint(*cloud3, 3.3, 3.3, 3.3);

	VisualPointFeatureVector3DPtr vector1 = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr vector2 = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr vector3 = NewVisualPointFeatureVector3D();

	AddPoint(*vector1, 1, 1, 1);
	AddPoint(*vector2, 2, 2, 2);
	AddPoint(*vector3, 3, 3, 3);

	Pose3DPtr origin1 = NewPose3D();
	Pose3DPtr origin2 = NewPose3D();
	Pose3DPtr pose1 = NewPose3D();
	Pose3DPtr pose2 = NewPose3D();
	Pose3DPtr pose3 = NewPose3D();
	
	SetPosition(*origin1, 0, 0, 0);
	SetPosition(*origin2, 1, 1, 1);
	SetPosition(*pose1, 0, 0, 0);
	SetPosition(*pose2, 1, 1, 1);
	SetPosition(*pose3, 1, 1, 1);

	SetOrientation(*origin1, 0, 0, 0, 1);
	SetOrientation(*origin2, 0, 0, 1, 0);
	SetOrientation(*pose1, 0, 0, 0, 1);
	SetOrientation(*pose2, 0, 0, 1, 0);
	SetOrientation(*pose3, 0, 0, 1, 0);

	map->AddPointCloud(cloud1, vector1, pose1);

	const Pose3D& latestPose1 = map->GetLatestPose();
	CLOSE_POSITION(latestPose1, 0, 0, 0);
	CLOSE_ORIENTATION(latestPose1, 0, 0, 0, 1);

	PointCloudConstPtr sceneCloudBase1 = map->GetScenePointCloud(origin1, 100);
	REQUIRE( GetNumberOfPoints(*sceneCloudBase1) == 2 );
	CLOSE_POINT( *sceneCloudBase1, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloudBase1, 1, 1.1, 1.1, 1.1);

	map->AttachPointCloud(cloud2, vector2, pose2);

	const Pose3D& latestPose2 = map->GetLatestPose();
	CLOSE_POSITION(latestPose2, 1, 1, 1);
	CLOSE_ORIENTATION(latestPose2, 0, 0, 1, 0);

	PointCloudConstPtr sceneCloudBase2 = map->GetScenePointCloud(origin1, 100);
	REQUIRE( GetNumberOfPoints(*sceneCloudBase2) == 4 );
	CLOSE_POINT( *sceneCloudBase2, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloudBase2, 1, 1.1, 1.1, 1.1);
	CLOSE_POINT( *sceneCloudBase2, 2, -1, -1, 1);
	CLOSE_POINT( *sceneCloudBase2, 3, -1.2, -1.2, 1.2);

	map->AttachPointCloud(cloud3, vector3, pose3);

	const Pose3D& latestPose3 = map->GetLatestPose();
	CLOSE_POSITION(latestPose3, 2, 2, 2);
	CLOSE_ORIENTATION(latestPose3, 0, 0, 0, 1);

	PointCloudConstPtr sceneCloud1 = map->GetScenePointCloud(origin1, 1.74);
	REQUIRE( GetNumberOfPoints(*sceneCloud1) == 2 );
	CLOSE_POINT( *sceneCloud1, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloud1, 1, -1, -1, 1);

	PointCloudConstPtr sceneCloud2 = map->GetScenePointCloud(origin1, 8.00);
	REQUIRE( GetNumberOfPoints(*sceneCloud2) == 5 );
	CLOSE_POINT( *sceneCloud2, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloud2, 1, 1.1, 1.1, 1.1);
	CLOSE_POINT( *sceneCloud2, 2, -1, -1, 1);
	CLOSE_POINT( *sceneCloud2, 3, -1.2, -1.2, 1.2);
	CLOSE_POINT( *sceneCloud2, 4, 1.3, 1.3, 1.3);

	PointCloudConstPtr sceneCloud3 = map->GetScenePointCloudInOrigin(origin1, 1.74);
	REQUIRE( GetNumberOfPoints(*sceneCloud3) == 2 );
	CLOSE_POINT( *sceneCloud3, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloud3, 1, -1, -1, 1);

	PointCloudConstPtr sceneCloud4 = map->GetScenePointCloudInOrigin(origin2, 8.00);
	REQUIRE( GetNumberOfPoints(*sceneCloud4) == 5 );
	CLOSE_POINT( *sceneCloud4, 0, 0, 0, 0);
	CLOSE_POINT( *sceneCloud4, 1, -0.1, -0.1, 0.1);
	CLOSE_POINT( *sceneCloud4, 2, -2, -2, 0);
	CLOSE_POINT( *sceneCloud4, 3, -2.2, -2.2, 0.2);
	CLOSE_POINT( *sceneCloud4, 4, -0.3, -0.3, 0.3);

	VisualPointFeatureVector3DConstPtr featureCloud1 = map->GetSceneFeaturesVector(origin1, 1.74);
	REQUIRE( GetNumberOfPoints(*featureCloud1) == 2 );
	CLOSE_POINT( *featureCloud1, 0, 1, 1, 1);
	CLOSE_POINT( *featureCloud1, 1, -1, -1, 1);

	VisualPointFeatureVector3DConstPtr featureCloud2 = map->GetSceneFeaturesVector(origin1, 6.00);
	REQUIRE( GetNumberOfPoints(*featureCloud2) == 2 );
	CLOSE_POINT( *featureCloud2, 0, 1, 1, 1);
	CLOSE_POINT( *featureCloud2, 1, -1, -1, 1);

	delete(map);
	delete(cloud1);
	delete(cloud2);
	delete(cloud3);
	delete(origin1);
	delete(origin2);
	delete(pose1);
	delete(pose2);
	delete(pose3);
	delete(vector1);
	delete(vector2);
	delete(vector3);

	delete(sceneCloud1);
	delete(sceneCloud2);
	delete(sceneCloud3);
	delete(sceneCloud4);
	delete(featureCloud1);
	delete(featureCloud2);
	}


TEST_CASE( "Add and Get 3 (PointCloudMap)", "[AddAndGet]" ) 
	{
	PointCloudMap* map = new PointCloudMap();

	PointCloudPtr cloud1 = NewPointCloud();
	PointCloudPtr cloud2 = NewPointCloud();
	PointCloudPtr cloud3 = NewPointCloud();	

	AddPoint(*cloud1, 1, 1, 1);
	AddPoint(*cloud1, 1.1, 1.1, 1.1);
	AddPoint(*cloud2, 2, 2, 2);
	AddPoint(*cloud2, 2.2, 2.2, 2.2);
	AddPoint(*cloud3, 3, 3, 3);
	AddPoint(*cloud3, 3.3, 3.3, 3.3);

	VisualPointFeatureVector3DPtr vector1 = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr vector2 = NewVisualPointFeatureVector3D();
	VisualPointFeatureVector3DPtr vector3 = NewVisualPointFeatureVector3D();

	AddPoint(*vector1, 1, 1, 1);
	AddPoint(*vector2, 2, 2, 2);
	AddPoint(*vector3, 3, 3, 3);

	Pose3DPtr origin1 = NewPose3D();
	Pose3DPtr origin2 = NewPose3D();
	Pose3DPtr pose1 = NewPose3D();
	Pose3DPtr pose2 = NewPose3D();
	Pose3DPtr pose3 = NewPose3D();
	
	SetPosition(*origin1, 0, 0, 0);
	SetPosition(*origin2, 1, 1, 1);
	SetPosition(*pose1, 0, 0, 0);
	SetPosition(*pose2, 1, 1, 1);
	SetPosition(*pose3, 1, 1, 1);

	SetOrientation(*origin1, 0, 0, 0, 1);
	SetOrientation(*origin2, 0, 0, 1, 0);
	SetOrientation(*pose1, 0, 0, 0, 1);
	SetOrientation(*pose2, 0, 0, 1, 0);
	SetOrientation(*pose3, 0, 0, 1, 0);

	map->AddPointCloud(cloud1, vector1, pose1);

	const Pose3D& latestPose1 = map->GetLatestPose();
	CLOSE_POSITION(latestPose1, 0, 0, 0);
	CLOSE_ORIENTATION(latestPose1, 0, 0, 0, 1);

	PointCloudConstPtr sceneCloudBase1 = map->GetScenePointCloud(origin1, 100);
	REQUIRE( GetNumberOfPoints(*sceneCloudBase1) == 2 );
	CLOSE_POINT( *sceneCloudBase1, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloudBase1, 1, 1.1, 1.1, 1.1);

	map->AddPointCloud(cloud2, vector2, pose2);

	const Pose3D& latestPose2 = map->GetLatestPose();
	CLOSE_POSITION(latestPose2, 1, 1, 1);
	CLOSE_ORIENTATION(latestPose2, 0, 0, 1, 0);

	PointCloudConstPtr sceneCloudBase2 = map->GetScenePointCloud(origin1, 100);
	REQUIRE( GetNumberOfPoints(*sceneCloudBase2) == 4 );
	CLOSE_POINT( *sceneCloudBase2, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloudBase2, 1, 1.1, 1.1, 1.1);
	CLOSE_POINT( *sceneCloudBase2, 2, -1, -1, 1);
	CLOSE_POINT( *sceneCloudBase2, 3, -1.2, -1.2, 1.2);

	map->AddPointCloud(cloud3, vector3, pose3);

	const Pose3D& latestPose3 = map->GetLatestPose();
	CLOSE_POSITION(latestPose3, 1, 1, 1);
	CLOSE_ORIENTATION(latestPose3, 0, 0, 1, 0);

	PointCloudConstPtr sceneCloud1 = map->GetScenePointCloud(origin1, 1.74);
	REQUIRE( GetNumberOfPoints(*sceneCloud1) == 2 );
	CLOSE_POINT( *sceneCloud1, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloud1, 1, -1, -1, 1);

	PointCloudConstPtr sceneCloud2 = map->GetScenePointCloud(origin1, 8.00);
	REQUIRE( GetNumberOfPoints(*sceneCloud2) == 6 );
	CLOSE_POINT( *sceneCloud2, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloud2, 1, 1.1, 1.1, 1.1);
	CLOSE_POINT( *sceneCloud2, 2, -1, -1, 1);
	CLOSE_POINT( *sceneCloud2, 3, -1.2, -1.2, 1.2);
	CLOSE_POINT( *sceneCloud2, 4, -2, -2, 2);
	CLOSE_POINT( *sceneCloud2, 5, -2.3, -2.3, 2.3);

	PointCloudConstPtr sceneCloud3 = map->GetScenePointCloudInOrigin(origin1, 1.74);
	REQUIRE( GetNumberOfPoints(*sceneCloud3) == 2 );
	CLOSE_POINT( *sceneCloud3, 0, 1, 1, 1);
	CLOSE_POINT( *sceneCloud3, 1, -1, -1, 1);

	PointCloudConstPtr sceneCloud4 = map->GetScenePointCloudInOrigin(origin2, 20.00);
	REQUIRE( GetNumberOfPoints(*sceneCloud4) == 6 );
	CLOSE_POINT( *sceneCloud4, 0, 0, 0, 0);
	CLOSE_POINT( *sceneCloud4, 1, -0.1, -0.1, 0.1);
	CLOSE_POINT( *sceneCloud4, 2, -2, -2, 0);
	CLOSE_POINT( *sceneCloud4, 3, -2.2, -2.2, 0.2);
	CLOSE_POINT( *sceneCloud2, 4, -3, -3, 1);
	CLOSE_POINT( *sceneCloud2, 5, -3.3, -3.3, 1.3);

	VisualPointFeatureVector3DConstPtr featureCloud1 = map->GetSceneFeaturesVector(origin1, 1.74);
	REQUIRE( GetNumberOfPoints(*featureCloud1) == 2 );
	CLOSE_POINT( *featureCloud1, 0, 1, 1, 1);
	CLOSE_POINT( *featureCloud1, 1, -1, -1, 1);

	VisualPointFeatureVector3DConstPtr featureCloud2 = map->GetSceneFeaturesVector(origin1, 20.00);
	REQUIRE( GetNumberOfPoints(*featureCloud2) == 3 );
	CLOSE_POINT( *featureCloud2, 0, 1, 1, 1);
	CLOSE_POINT( *featureCloud2, 1, -1, -1, 1);
	CLOSE_POINT( *featureCloud2, 1, -2, -2, 2);

	delete(map);
	delete(cloud1);
	delete(cloud2);
	delete(cloud3);
	delete(origin1);
	delete(origin2);
	delete(pose1);
	delete(pose2);
	delete(pose3);
	delete(vector1);
	delete(vector2);
	delete(vector3);

	delete(sceneCloud1);
	delete(sceneCloud2);
	delete(sceneCloud3);
	delete(sceneCloud4);
	delete(featureCloud1);
	delete(featureCloud2);
	}
