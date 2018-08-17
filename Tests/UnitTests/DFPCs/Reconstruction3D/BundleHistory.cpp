/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file BundleHistory.cpp
 * @date 15/08/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the BundleHistory Class.
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
#include <Reconstruction3D/BundleHistory.hpp>
#include <Errors/Assert.hpp>

using namespace FrameWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace VisualPointFeatureVector3DWrapper;
using namespace CorrespondenceMap2DWrapper;
using namespace PointCloudWrapper;
using namespace BaseTypesWrapper;
using namespace dfpc_ci;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */


TEST_CASE( "Add and Get (BundleHistory)", "[AddAndGet]" ) 
	{
	BundleHistory* bundleHistory = new BundleHistory(2);

	FramePtr leftFrame = NewFrame();
	FramePtr rightFrame = NewFrame();
	VisualPointFeatureVector2DPtr leftFeature = NewVisualPointFeatureVector2D();
	VisualPointFeatureVector2DPtr rightFeature = NewVisualPointFeatureVector2D();
	VisualPointFeatureVector3DPtr cloudFeature = NewVisualPointFeatureVector3D();
	CorrespondenceMap2DPtr correspondence1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr correspondence2 = NewCorrespondenceMap2D();
	PointCloudPtr pointCloud1 = NewPointCloud();
	PointCloudPtr pointCloud2 = NewPointCloud();

	SetFrameSize(*leftFrame, 1, 1);
	SetFrameSize(*rightFrame, 2, 1);
	AppendData(*leftFrame, byte(0x2) );
	AppendData(*rightFrame, byte(0x3) );
	AppendData(*rightFrame, byte(0x3) );
	
	AddPoint(*leftFeature, 1, 1);
	AddPoint(*rightFeature, 2, 2);
	AddPoint(*cloudFeature, 3, 3, 3);
	
	Point2D source1, source2, sink1, sink2;
	source1.x = 1; source1.y = 1;
	source2.x = 2; source2.y = 2;
	sink1.x = 3; sink1.y = 3;
	sink2.x = 4; sink2.y = 4;
	AddCorrespondence(*correspondence1, source1, sink1, 1);
	AddCorrespondence(*correspondence2, source2, sink2, 1);

	AddPoint(*pointCloud1, 1, 1, 1);
	AddPoint(*pointCloud2, 2, 2, 2);


	bundleHistory->AddImages(*leftFrame, *rightFrame);
	bundleHistory->AddFeatures(*leftFeature, "first");
	bundleHistory->AddFeatures(*rightFeature, "second");
	bundleHistory->AddFeatures3d(*cloudFeature, "first");
	bundleHistory->AddMatches(*correspondence1, "one");
	bundleHistory->AddMatches(*correspondence2, "two");
	bundleHistory->AddPointCloud(*pointCloud1, "one");	
	bundleHistory->AddPointCloud(*pointCloud2, "two");

	bundleHistory->AddImages(*leftFrame, *leftFrame);
	bundleHistory->AddPointCloud(*pointCloud2, "second");
	bundleHistory->AddPointCloud(*pointCloud1, "second");

	FrameConstPtr frame1 = bundleHistory->GetLeftImage(0);
	FrameConstPtr frame2 = bundleHistory->GetRightImage(0);
	VisualPointFeatureVector2DConstPtr features1 = bundleHistory->GetFeatures(0, "first");
	VisualPointFeatureVector2DConstPtr features2 = bundleHistory->GetFeatures(0, "second");
	VisualPointFeatureVector3DConstPtr features3d = bundleHistory->GetFeatures3d(0, "second");
	CorrespondenceMap2DConstPtr map1 = bundleHistory->GetMatches(0, "one");
	CorrespondenceMap2DConstPtr map2 = bundleHistory->GetMatches(0, "two");
	PointCloudConstPtr cloud1 = bundleHistory->GetPointCloud(0, "first");
	PointCloudConstPtr cloud2 = bundleHistory->GetPointCloud(0, "second");

	REQUIRE( frame1 != NULL );
	REQUIRE( GetFrameWidth(*frame1)  == 1 );
	REQUIRE( GetDataByte(*frame1, 0) == byte(0x2) );
	REQUIRE( frame2 != NULL );
	REQUIRE( GetFrameWidth(*frame2)  == 1 );
	REQUIRE( GetDataByte(*frame2, 0) == byte(0x2) );
	REQUIRE( features1 == NULL );
	REQUIRE( features2 == NULL );
	REQUIRE( features3d == NULL );
	REQUIRE( map1 == NULL );
	REQUIRE( map2 == NULL );
	REQUIRE( cloud1 == NULL );
	REQUIRE( cloud2 != NULL );
	REQUIRE( GetNumberOfPoints(*cloud2) == 1 );
	REQUIRE( GetXCoordinate(*cloud2, 0) == 1 );

	FrameConstPtr frame3 = bundleHistory->GetLeftImage(1);
	FrameConstPtr frame4 = bundleHistory->GetRightImage(1);
	VisualPointFeatureVector2DConstPtr features3 = bundleHistory->GetFeatures(1, "first");
	VisualPointFeatureVector2DConstPtr features4 = bundleHistory->GetFeatures(1, "second");
	VisualPointFeatureVector3DConstPtr features5 = bundleHistory->GetFeatures3d(1, "first");
	CorrespondenceMap2DConstPtr map3 = bundleHistory->GetMatches(1, "one");
	CorrespondenceMap2DConstPtr map4 = bundleHistory->GetMatches(1, "two");
	PointCloudConstPtr cloud3 = bundleHistory->GetPointCloud(1, "first");
	PointCloudConstPtr cloud4 = bundleHistory->GetPointCloud(1, "two");


	REQUIRE( frame3 != NULL );
	REQUIRE( GetFrameWidth(*frame3)  == 1 );
	REQUIRE( GetDataByte(*frame3, 0) == byte(0x2) );
	REQUIRE( frame4 != NULL );
	REQUIRE( GetFrameWidth(*frame4)  == 2 );
	REQUIRE( GetDataByte(*frame4, 0) == byte(0x3) );
	REQUIRE( features3 != NULL );
	REQUIRE( GetNumberOfPoints(*features3) == 1 );
	REQUIRE( GetXCoordinate(*features3, 0) == 1 );
	REQUIRE( features4 != NULL );
	REQUIRE( GetNumberOfPoints(*features4) == 1 );
	REQUIRE( GetXCoordinate(*features4, 0) == 2 );
	REQUIRE( features5 != NULL );
	REQUIRE( GetNumberOfPoints(*features5) == 1 );
	REQUIRE( GetXCoordinate(*features5, 0) == 3 );
	REQUIRE( map3 != NULL );
	REQUIRE( GetNumberOfCorrespondences(*map3) == 1 );
	REQUIRE( GetSource(*map3, 0).x == 1 );
	REQUIRE( map4 != NULL );
	REQUIRE( GetNumberOfCorrespondences(*map4) == 1 );
	REQUIRE( GetSource(*map4, 0).x == 2 );
	REQUIRE( cloud3 == NULL );
	REQUIRE( cloud4 != NULL );
	REQUIRE( GetNumberOfPoints(*cloud4) == 1 );
	REQUIRE( GetXCoordinate(*cloud4, 0) == 2 );

	bundleHistory->RemoveEntry(0);

	frame3 = bundleHistory->GetLeftImage(0);
	frame4 = bundleHistory->GetRightImage(0);
	features3 = bundleHistory->GetFeatures(0, "first");
	features4 = bundleHistory->GetFeatures(0, "second");
	VisualPointFeatureVector3DConstPtr features8 = bundleHistory->GetFeatures3d(0, "first");
	map3 = bundleHistory->GetMatches(0, "one");
	map4 = bundleHistory->GetMatches(0, "two");
	cloud3 = bundleHistory->GetPointCloud(0, "first");
	cloud4 = bundleHistory->GetPointCloud(0, "two");


	REQUIRE( frame3 != NULL );
	REQUIRE( GetFrameWidth(*frame3)  == 1 );
	REQUIRE( GetDataByte(*frame3, 0) == byte(0x2) );
	REQUIRE( frame4 != NULL );
	REQUIRE( GetFrameWidth(*frame4)  == 2 );
	REQUIRE( GetDataByte(*frame4, 0) == byte(0x3) );
	REQUIRE( features3 != NULL );
	REQUIRE( GetNumberOfPoints(*features3) == 1 );
	REQUIRE( GetXCoordinate(*features3, 0) == 1 );
	REQUIRE( features4 != NULL );
	REQUIRE( GetNumberOfPoints(*features4) == 1 );
	REQUIRE( GetXCoordinate(*features4, 0) == 2 );
	REQUIRE( features8 != NULL );
	REQUIRE( GetNumberOfPoints(*features8) == 1 );
	REQUIRE( GetXCoordinate(*features8, 0) == 3 );
	REQUIRE( map3 != NULL );
	REQUIRE( GetNumberOfCorrespondences(*map3) == 1 );
	REQUIRE( GetSource(*map3, 0).x == 1 );
	REQUIRE( map4 != NULL );
	REQUIRE( GetNumberOfCorrespondences(*map4) == 1 );
	REQUIRE( GetSource(*map4, 0).x == 2 );
	REQUIRE( cloud3 == NULL );
	REQUIRE( cloud4 != NULL );
	REQUIRE( GetNumberOfPoints(*cloud4) == 1 );
	REQUIRE( GetXCoordinate(*cloud4, 0) == 2 );
	

	delete(bundleHistory);
	delete(leftFrame);
	delete(rightFrame);
	delete(leftFeature);
	delete(rightFeature);
	delete(cloudFeature);
	delete(correspondence1);
	delete(correspondence2);
	delete(pointCloud1);
	delete(pointCloud2);
	}

