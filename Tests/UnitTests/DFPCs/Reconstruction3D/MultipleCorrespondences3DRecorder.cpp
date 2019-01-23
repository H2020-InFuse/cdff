/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MultipleCorrespondences3DRecorder.cpp
 * @date 15/08/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the MultipleCorrespondences3DRecorder Class.
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
#include <Reconstruction3D/MultipleCorrespondences3DRecorder.hpp>
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap2DWrapper;
using namespace CorrespondenceMap3DWrapper;
using namespace PointCloudWrapper;
using namespace CDFF::DFPC::Reconstruction3D;
using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Add and Get (MultipleCorrespondences3DRecorder)", "[AddAndGet]" )
	{
	CorrespondenceMap2DPtr map1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr map2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr map3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr map4 = NewCorrespondenceMap2D();
	PointCloudPtr cloud1 = NewPointCloud();
	PointCloudPtr cloud2 = NewPointCloud();

	Point2D point1, point2, point3, point4;
	point1.x = 1; point1.y = 1;
	point2.x = 2; point2.y = 2;
	point3.x = 3; point3.y = 3;
	point4.x = 4; point4.y = 4;
	AddCorrespondence(*map1, point1, point2, 1);
	AddCorrespondence(*map2, point1, point3, 1);
	AddCorrespondence(*map3, point2, point4, 1);
	AddCorrespondence(*map4, point3, point4, 1);
	AddCorrespondence(*map4, point1, point1, 1);

	AddPoint(*cloud1, 0, 0, 0);
	AddPoint(*cloud2, 1, 1, 1);
	AddPoint(*cloud2, 1, 1, 5);


	CorrespondenceMap2DPtr map5 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr map6 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr map7 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr map8 = NewCorrespondenceMap2D();
	PointCloudPtr cloud3 = NewPointCloud();
	PointCloudPtr cloud4 = NewPointCloud();

	Point2D point5, point6, point7, point8;
	point5.x = 5; point5.y = 5;
	point6.x = 6; point6.y = 6;
	point7.x = 7; point7.y = 7;
	point8.x = 8; point8.y = 8;
	AddCorrespondence(*map5, point5, point6, 1);
	AddCorrespondence(*map6, point5, point7, 1);
	AddCorrespondence(*map7, point6, point8, 1);
	AddCorrespondence(*map8, point7, point8, 1);
	AddCorrespondence(*map8, point5, point5, 1);

	AddPoint(*cloud3, 2, 2, 2);
	AddPoint(*cloud4, 3, 3, 3);
	AddPoint(*cloud4, 3, 6, 3);

	std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> mapList = {map1, map2, map3, map4};
	std::vector<PointCloudWrapper::PointCloudConstPtr> pointCloudList = {cloud1, cloud2};
	std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> mapList2 = {map5, map6, map7, map8};
	std::vector<PointCloudWrapper::PointCloudConstPtr> pointCloudList2 = {cloud3, cloud4};


	MultipleCorrespondences3DRecorder* recorder = new MultipleCorrespondences3DRecorder(3);
	CorrespondenceMaps3DSequencePtr sequenceM1 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequenceM1) == 0);

	recorder->InitializeNewSequence();
	recorder->CompleteNewSequence();
	CorrespondenceMaps3DSequencePtr sequence0 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence0) == 0);

	recorder->InitializeNewSequence();
	recorder->AddCorrespondencesFromTwoImagePairs(mapList, pointCloudList);
	recorder->CompleteNewSequence();

	CorrespondenceMaps3DSequencePtr sequence1 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence1) == 1);
	const CorrespondenceMap3D& refMap1 = GetCorrespondenceMap(*sequence1, 0);
	REQUIRE( GetNumberOfCorrespondences(refMap1) == 1);
	REQUIRE( GetSource(refMap1, 0).x == 0);
	REQUIRE( GetSink(refMap1, 0).x == 1);

	recorder->DiscardLatestCorrespondences();
	CorrespondenceMaps3DSequencePtr sequence2 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence2) == 0);

	recorder->InitializeNewSequence();
	recorder->AddCorrespondencesFromTwoImagePairs(mapList, pointCloudList);
	recorder->CompleteNewSequence();

	CorrespondenceMaps3DSequencePtr sequence3 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence3) == 1);
	const CorrespondenceMap3D& refMap2 = GetCorrespondenceMap(*sequence3, 0);
	REQUIRE( GetNumberOfCorrespondences(refMap2) == 1);
	REQUIRE( GetSource(refMap2, 0).x == 0);
	REQUIRE( GetSink(refMap2, 0).x == 1);

	recorder->InitializeNewSequence();
	recorder->AddCorrespondencesFromTwoImagePairs(mapList, pointCloudList);
	recorder->AddCorrespondencesFromTwoImagePairs(mapList, pointCloudList);
	recorder->CompleteNewSequence();

	CorrespondenceMaps3DSequencePtr sequence4 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence4) == 3);
	const CorrespondenceMap3D& refMap3 = GetCorrespondenceMap(*sequence4, 0);
	const CorrespondenceMap3D& refMap4 = GetCorrespondenceMap(*sequence4, 1);
	const CorrespondenceMap3D& refMap5 = GetCorrespondenceMap(*sequence4, 2);
	REQUIRE( GetNumberOfCorrespondences(refMap3) == 1);
	REQUIRE( GetSource(refMap3, 0).x == 0);
	REQUIRE( GetSink(refMap3, 0).x == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4) == 1);
	REQUIRE( GetSource(refMap4, 0).x == 0);
	REQUIRE( GetSink(refMap4, 0).x == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5) == 1);
	REQUIRE( GetSource(refMap5, 0).x == 0);
	REQUIRE( GetSink(refMap5, 0).x == 1);

	recorder->InitializeNewSequence();
	recorder->AddCorrespondencesFromTwoImagePairs(mapList2, pointCloudList2);
	recorder->AddCorrespondencesFromTwoImagePairs(mapList2, pointCloudList2);
	recorder->CompleteNewSequence();

	CorrespondenceMaps3DSequencePtr sequence5 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence5) == 3);
	const CorrespondenceMap3D& refMap6 = GetCorrespondenceMap(*sequence5, 0);
	const CorrespondenceMap3D& refMap7 = GetCorrespondenceMap(*sequence5, 1);
	const CorrespondenceMap3D& refMap8 = GetCorrespondenceMap(*sequence5, 2);
	REQUIRE( GetNumberOfCorrespondences(refMap6) == 1);
	REQUIRE( GetSource(refMap6, 0).x == 2);
	REQUIRE( GetSink(refMap6, 0).x == 3);
	REQUIRE( GetNumberOfCorrespondences(refMap7) == 1);
	REQUIRE( GetSource(refMap7, 0).x == 2);
	REQUIRE( GetSink(refMap7, 0).x == 3);
	REQUIRE( GetNumberOfCorrespondences(refMap8) == 1);
	REQUIRE( GetSource(refMap8, 0).x == 0);
	REQUIRE( GetSink(refMap8, 0).x == 1);

	recorder->DiscardLatestCorrespondences();

	CorrespondenceMaps3DSequencePtr sequence6 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence6) == 3);
	const CorrespondenceMap3D& refMap9 = GetCorrespondenceMap(*sequence6, 0);
	const CorrespondenceMap3D& refMap10 = GetCorrespondenceMap(*sequence6, 1);
	const CorrespondenceMap3D& refMap11 = GetCorrespondenceMap(*sequence6, 2);
	REQUIRE( GetNumberOfCorrespondences(refMap9) == 1);
	REQUIRE( GetSource(refMap9, 0).x == 0);
	REQUIRE( GetSink(refMap9, 0).x == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap10) == 1);
	REQUIRE( GetSource(refMap10, 0).x == 0);
	REQUIRE( GetSink(refMap10, 0).x == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap11) == 1);
	REQUIRE( GetSource(refMap11, 0).x == 0);
	REQUIRE( GetSink(refMap11, 0).x == 1);

	delete(recorder);
	delete(map1);
	delete(map2);
	delete(map3);
	delete(map4);
	delete(cloud1);
	delete(cloud2);
	delete(map5);
	delete(map6);
	delete(map7);
	delete(map8);
	delete(cloud3);
	delete(cloud4);
	}
