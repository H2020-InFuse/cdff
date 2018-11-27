/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MultipleCorrespondences2DRecorder.cpp
 * @date 15/08/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the MultipleCorrespondences2DRecorder Class.
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
#include <Reconstruction3D/MultipleCorrespondences2DRecorder.hpp>
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap2DWrapper;
using namespace PointCloudWrapper;
using namespace CDFF::DFPC::Reconstruction3D;
using namespace BaseTypesWrapper;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Add and Get (MultipleCorrespondences2DRecorder)", "[AddAndGet]" )
	{
	CorrespondenceMap2DPtr mapL1R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL2R2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL2L1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL2R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR2L1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR2R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3R3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3L2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3R2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3L1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR3L2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR3R2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR3L1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR3R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4R4 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4L3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4R3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4L2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4R2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR4L3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR4R3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR4L2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR4R2 = NewCorrespondenceMap2D();

	Point2D left1, right1, left2, right2, left3, right3, left4, right4;
	left1.x = 1; left1.y = 1; right1.x = 2; right1.y = 2;
	left2.x = 3; left2.y = 3; right2.x = 4; right2.y = 4;
	left3.x = 5; left3.y = 5; right3.x = 6; right3.y = 6;
	left4.x = 7; left4.y = 7; right4.x = 8; right4.y = 8;
	AddCorrespondence(*mapL1R1, left1, right1, 1);
	AddCorrespondence(*mapL2R2, left2, right2, 1);
	AddCorrespondence(*mapL2L1, left2, left1, 1);
	AddCorrespondence(*mapL2R1, left2, right1, 1);
	AddCorrespondence(*mapR2L1, right2, left1, 1);
	AddCorrespondence(*mapR2R1, right2, right1, 1);
	AddCorrespondence(*mapL3R3, left3, right3, 1);
	AddCorrespondence(*mapL3L2, left3, left2, 1);
	AddCorrespondence(*mapL3R2, left3, right2, 1);
	AddCorrespondence(*mapL3L1, left3, left1, 1);
	AddCorrespondence(*mapL3R1, left3, right1, 1);
	AddCorrespondence(*mapR3L2, right3, left2, 1);
	AddCorrespondence(*mapR3R2, right3, right2, 1);
	AddCorrespondence(*mapR3L1, right3, left1, 1);
	AddCorrespondence(*mapR3R1, right3, right1, 1);
	AddCorrespondence(*mapL4R4, left4, right4, 1);
	AddCorrespondence(*mapL4L3, left4, left3, 1);
	AddCorrespondence(*mapL4R3, left4, right3, 1);
	AddCorrespondence(*mapL4L2, left4, left2, 1);
	AddCorrespondence(*mapL4R2, left4, right2, 1);
	AddCorrespondence(*mapR4L3, right4, left3, 1);
	AddCorrespondence(*mapR4R3, right4, right3, 1);
	AddCorrespondence(*mapR4L2, right4, left2, 1);
	AddCorrespondence(*mapR4R2, right4, right2, 1);


	MultipleCorrespondences2DRecorder* recorder = new MultipleCorrespondences2DRecorder(3);
	CorrespondenceMaps2DSequenceConstPtr sequenceM1 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequenceM1) == 0);

	recorder->InitializeNewSequence();
	recorder->AddCorrespondences(mapL1R1);
	recorder->CompleteNewSequence();
	CorrespondenceMaps2DSequenceConstPtr sequence0 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence0) == 1);
	const CorrespondenceMap2D& refMap0 = GetCorrespondenceMap(*sequence0, 0);
	REQUIRE( GetNumberOfCorrespondences(refMap0) == 1);
	REQUIRE( GetSource(refMap0, 0).x == 1);
	REQUIRE( GetSink(refMap0, 0).x == 2);	

	recorder->InitializeNewSequence();
	recorder->AddCorrespondences(mapL2R2);
	recorder->AddCorrespondences(mapL2L1);
	recorder->AddCorrespondences(mapL2R1);
	recorder->AddCorrespondences(mapR2L1);
	recorder->AddCorrespondences(mapR2R1);
	recorder->CompleteNewSequence();

	CorrespondenceMaps2DSequenceConstPtr sequence1 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence1) == 6);
	const CorrespondenceMap2D& refMap1A = GetCorrespondenceMap(*sequence1, 0);
	const CorrespondenceMap2D& refMap1B = GetCorrespondenceMap(*sequence1, 1);
	const CorrespondenceMap2D& refMap1C = GetCorrespondenceMap(*sequence1, 2);
	const CorrespondenceMap2D& refMap1D = GetCorrespondenceMap(*sequence1, 3);
	const CorrespondenceMap2D& refMap1E = GetCorrespondenceMap(*sequence1, 4);
	const CorrespondenceMap2D& refMap1F = GetCorrespondenceMap(*sequence1, 5);
	REQUIRE( GetNumberOfCorrespondences(refMap1A) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap1B) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap1C) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap1D) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap1E) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap1F) == 1);
	REQUIRE( GetSource(refMap1A, 0).x == 3);
	REQUIRE( GetSource(refMap1B, 0).x == 3);
	REQUIRE( GetSource(refMap1C, 0).x == 3);
	REQUIRE( GetSource(refMap1D, 0).x == 4);
	REQUIRE( GetSource(refMap1E, 0).x == 4);
	REQUIRE( GetSource(refMap1F, 0).x == 1);
	REQUIRE( GetSink(refMap1A, 0).x == 4);
	REQUIRE( GetSink(refMap1B, 0).x == 1);
	REQUIRE( GetSink(refMap1C, 0).x == 2);
	REQUIRE( GetSink(refMap1D, 0).x == 1);
	REQUIRE( GetSink(refMap1E, 0).x == 2);
	REQUIRE( GetSink(refMap1F, 0).x == 2);

	recorder->DiscardLatestCorrespondences();
	CorrespondenceMaps2DSequenceConstPtr sequence2 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence2) == 1);
	const CorrespondenceMap2D& refMap2 = GetCorrespondenceMap(*sequence2, 0);
	REQUIRE( GetNumberOfCorrespondences(refMap2) == 1);
	REQUIRE( GetSource(refMap2, 0).x == 1);
	REQUIRE( GetSink(refMap2, 0).x == 2);

	recorder->InitializeNewSequence();
	recorder->AddCorrespondences(mapL2R2);
	recorder->AddCorrespondences(mapL2L1);
	recorder->AddCorrespondences(mapL2R1);
	recorder->AddCorrespondences(mapR2L1);
	recorder->AddCorrespondences(mapR2R1);
	recorder->CompleteNewSequence();

	CorrespondenceMaps2DSequenceConstPtr sequence3 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence3) == 6);
	const CorrespondenceMap2D& refMap2A = GetCorrespondenceMap(*sequence3, 0);
	const CorrespondenceMap2D& refMap2B = GetCorrespondenceMap(*sequence3, 1);
	const CorrespondenceMap2D& refMap2C = GetCorrespondenceMap(*sequence3, 2);
	const CorrespondenceMap2D& refMap2D = GetCorrespondenceMap(*sequence3, 3);
	const CorrespondenceMap2D& refMap2E = GetCorrespondenceMap(*sequence3, 4);
	const CorrespondenceMap2D& refMap2F = GetCorrespondenceMap(*sequence3, 5);
	REQUIRE( GetNumberOfCorrespondences(refMap2A) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap2B) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap2C) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap2D) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap2E) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap2F) == 1);
	REQUIRE( GetSource(refMap2A, 0).x == 3);
	REQUIRE( GetSource(refMap2B, 0).x == 3);
	REQUIRE( GetSource(refMap2C, 0).x == 3);
	REQUIRE( GetSource(refMap2D, 0).x == 4);
	REQUIRE( GetSource(refMap2E, 0).x == 4);
	REQUIRE( GetSource(refMap2F, 0).x == 1);
	REQUIRE( GetSink(refMap2A, 0).x == 4);
	REQUIRE( GetSink(refMap2B, 0).x == 1);
	REQUIRE( GetSink(refMap2C, 0).x == 2);
	REQUIRE( GetSink(refMap2D, 0).x == 1);
	REQUIRE( GetSink(refMap2E, 0).x == 2);
	REQUIRE( GetSink(refMap2F, 0).x == 2);

	recorder->InitializeNewSequence();
	recorder->AddCorrespondences(mapL3R3);
	recorder->AddCorrespondences(mapL3L2);
	recorder->AddCorrespondences(mapL3R2);
	recorder->AddCorrespondences(mapL3L1);
	recorder->AddCorrespondences(mapL3R1);
	recorder->AddCorrespondences(mapR3L2);
	recorder->AddCorrespondences(mapR3R2);
	recorder->AddCorrespondences(mapR3L1);
	recorder->AddCorrespondences(mapR3R1);
	recorder->CompleteNewSequence();

	CorrespondenceMaps2DSequenceConstPtr sequence4 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence4) == 15);
	const CorrespondenceMap2D& refMap3A = GetCorrespondenceMap(*sequence4, 0);
	const CorrespondenceMap2D& refMap3B = GetCorrespondenceMap(*sequence4, 1);
	const CorrespondenceMap2D& refMap3C = GetCorrespondenceMap(*sequence4, 2);
	const CorrespondenceMap2D& refMap3D = GetCorrespondenceMap(*sequence4, 3);
	const CorrespondenceMap2D& refMap3E = GetCorrespondenceMap(*sequence4, 4);
	const CorrespondenceMap2D& refMap3F = GetCorrespondenceMap(*sequence4, 5);
	const CorrespondenceMap2D& refMap3G = GetCorrespondenceMap(*sequence4, 6);
	const CorrespondenceMap2D& refMap3H = GetCorrespondenceMap(*sequence4, 7);
	const CorrespondenceMap2D& refMap3I = GetCorrespondenceMap(*sequence4, 8);
	const CorrespondenceMap2D& refMap3J = GetCorrespondenceMap(*sequence4, 9);
	const CorrespondenceMap2D& refMap3K = GetCorrespondenceMap(*sequence4, 10);
	const CorrespondenceMap2D& refMap3L = GetCorrespondenceMap(*sequence4, 11);
	const CorrespondenceMap2D& refMap3M = GetCorrespondenceMap(*sequence4, 12);
	const CorrespondenceMap2D& refMap3N = GetCorrespondenceMap(*sequence4, 13);
	const CorrespondenceMap2D& refMap3O = GetCorrespondenceMap(*sequence4, 14);
	REQUIRE( GetNumberOfCorrespondences(refMap3A) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3B) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3C) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3D) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3E) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3F) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3G) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3H) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3I) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3J) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3K) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3L) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3M) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3N) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap3O) == 1);
	REQUIRE( GetSource(refMap3A, 0).x == 5);
	REQUIRE( GetSource(refMap3B, 0).x == 5);
	REQUIRE( GetSource(refMap3C, 0).x == 5);
	REQUIRE( GetSource(refMap3D, 0).x == 5);
	REQUIRE( GetSource(refMap3E, 0).x == 5);
	REQUIRE( GetSource(refMap3F, 0).x == 6);
	REQUIRE( GetSource(refMap3G, 0).x == 6);
	REQUIRE( GetSource(refMap3H, 0).x == 6);
	REQUIRE( GetSource(refMap3I, 0).x == 6);
	REQUIRE( GetSource(refMap3J, 0).x == 3);
	REQUIRE( GetSource(refMap3K, 0).x == 3);
	REQUIRE( GetSource(refMap3L, 0).x == 3);
	REQUIRE( GetSource(refMap3M, 0).x == 4);
	REQUIRE( GetSource(refMap3N, 0).x == 4);
	REQUIRE( GetSource(refMap3O, 0).x == 1);
	REQUIRE( GetSink(refMap3A, 0).x == 6);
	REQUIRE( GetSink(refMap3B, 0).x == 3);
	REQUIRE( GetSink(refMap3C, 0).x == 4);
	REQUIRE( GetSink(refMap3D, 0).x == 1);
	REQUIRE( GetSink(refMap3E, 0).x == 2);
	REQUIRE( GetSink(refMap3F, 0).x == 3);
	REQUIRE( GetSink(refMap3G, 0).x == 4);
	REQUIRE( GetSink(refMap3H, 0).x == 1);
	REQUIRE( GetSink(refMap3I, 0).x == 2);
	REQUIRE( GetSink(refMap3J, 0).x == 4);
	REQUIRE( GetSink(refMap3K, 0).x == 1);
	REQUIRE( GetSink(refMap3L, 0).x == 2);
	REQUIRE( GetSink(refMap3M, 0).x == 1);
	REQUIRE( GetSink(refMap3N, 0).x == 2);
	REQUIRE( GetSink(refMap3O, 0).x == 2);

	recorder->InitializeNewSequence();
	recorder->AddCorrespondences(mapL4R4);
	recorder->AddCorrespondences(mapL4L3);
	recorder->AddCorrespondences(mapL4R3);
	recorder->AddCorrespondences(mapL4L2);
	recorder->AddCorrespondences(mapL4R2);
	recorder->AddCorrespondences(mapR4L3);
	recorder->AddCorrespondences(mapR4R3);
	recorder->AddCorrespondences(mapR4L2);
	recorder->AddCorrespondences(mapR4R2);
	recorder->CompleteNewSequence();

	CorrespondenceMaps2DSequenceConstPtr sequence5 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence5) == 15);
	const CorrespondenceMap2D& refMap4A = GetCorrespondenceMap(*sequence5, 0);
	const CorrespondenceMap2D& refMap4B = GetCorrespondenceMap(*sequence5, 1);
	const CorrespondenceMap2D& refMap4C = GetCorrespondenceMap(*sequence5, 2);
	const CorrespondenceMap2D& refMap4D = GetCorrespondenceMap(*sequence5, 3);
	const CorrespondenceMap2D& refMap4E = GetCorrespondenceMap(*sequence5, 4);
	const CorrespondenceMap2D& refMap4F = GetCorrespondenceMap(*sequence5, 5);
	const CorrespondenceMap2D& refMap4G = GetCorrespondenceMap(*sequence5, 6);
	const CorrespondenceMap2D& refMap4H = GetCorrespondenceMap(*sequence5, 7);
	const CorrespondenceMap2D& refMap4I = GetCorrespondenceMap(*sequence5, 8);
	const CorrespondenceMap2D& refMap4J = GetCorrespondenceMap(*sequence5, 9);
	const CorrespondenceMap2D& refMap4K = GetCorrespondenceMap(*sequence5, 10);
	const CorrespondenceMap2D& refMap4L = GetCorrespondenceMap(*sequence5, 11);
	const CorrespondenceMap2D& refMap4M = GetCorrespondenceMap(*sequence5, 12);
	const CorrespondenceMap2D& refMap4N = GetCorrespondenceMap(*sequence5, 13);
	const CorrespondenceMap2D& refMap4O = GetCorrespondenceMap(*sequence5, 14);
	REQUIRE( GetNumberOfCorrespondences(refMap4A) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4B) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4C) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4D) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4E) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4F) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4G) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4H) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4I) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4J) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4K) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4L) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4M) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4N) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap4O) == 1);
	REQUIRE( GetSource(refMap4A, 0).x == 7);
	REQUIRE( GetSource(refMap4B, 0).x == 7);
	REQUIRE( GetSource(refMap4C, 0).x == 7);
	REQUIRE( GetSource(refMap4D, 0).x == 7);
	REQUIRE( GetSource(refMap4E, 0).x == 7);
	REQUIRE( GetSource(refMap4F, 0).x == 8);
	REQUIRE( GetSource(refMap4G, 0).x == 8);
	REQUIRE( GetSource(refMap4H, 0).x == 8);
	REQUIRE( GetSource(refMap4I, 0).x == 8);
	REQUIRE( GetSource(refMap4J, 0).x == 5);
	REQUIRE( GetSource(refMap4K, 0).x == 5);
	REQUIRE( GetSource(refMap4L, 0).x == 5);
	REQUIRE( GetSource(refMap4M, 0).x == 6);
	REQUIRE( GetSource(refMap4N, 0).x == 6);
	REQUIRE( GetSource(refMap4O, 0).x == 3);
	REQUIRE( GetSink(refMap4A, 0).x == 8);
	REQUIRE( GetSink(refMap4B, 0).x == 5);
	REQUIRE( GetSink(refMap4C, 0).x == 6);
	REQUIRE( GetSink(refMap4D, 0).x == 3);
	REQUIRE( GetSink(refMap4E, 0).x == 4);
	REQUIRE( GetSink(refMap4F, 0).x == 5);
	REQUIRE( GetSink(refMap4G, 0).x == 6);
	REQUIRE( GetSink(refMap4H, 0).x == 3);
	REQUIRE( GetSink(refMap4I, 0).x == 4);
	REQUIRE( GetSink(refMap4J, 0).x == 6);
	REQUIRE( GetSink(refMap4K, 0).x == 3);
	REQUIRE( GetSink(refMap4L, 0).x == 4);
	REQUIRE( GetSink(refMap4M, 0).x == 3);
	REQUIRE( GetSink(refMap4N, 0).x == 4);
	REQUIRE( GetSink(refMap4O, 0).x == 4);

	recorder->DiscardLatestCorrespondences();

	CorrespondenceMaps2DSequenceConstPtr sequence6 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence6) == 15);
	const CorrespondenceMap2D& refMap5A = GetCorrespondenceMap(*sequence6, 0);
	const CorrespondenceMap2D& refMap5B = GetCorrespondenceMap(*sequence6, 1);
	const CorrespondenceMap2D& refMap5C = GetCorrespondenceMap(*sequence6, 2);
	const CorrespondenceMap2D& refMap5D = GetCorrespondenceMap(*sequence6, 3);
	const CorrespondenceMap2D& refMap5E = GetCorrespondenceMap(*sequence6, 4);
	const CorrespondenceMap2D& refMap5F = GetCorrespondenceMap(*sequence6, 5);
	const CorrespondenceMap2D& refMap5G = GetCorrespondenceMap(*sequence6, 6);
	const CorrespondenceMap2D& refMap5H = GetCorrespondenceMap(*sequence6, 7);
	const CorrespondenceMap2D& refMap5I = GetCorrespondenceMap(*sequence6, 8);
	const CorrespondenceMap2D& refMap5J = GetCorrespondenceMap(*sequence6, 9);
	const CorrespondenceMap2D& refMap5K = GetCorrespondenceMap(*sequence6, 10);
	const CorrespondenceMap2D& refMap5L = GetCorrespondenceMap(*sequence6, 11);
	const CorrespondenceMap2D& refMap5M = GetCorrespondenceMap(*sequence6, 12);
	const CorrespondenceMap2D& refMap5N = GetCorrespondenceMap(*sequence6, 13);
	const CorrespondenceMap2D& refMap5O = GetCorrespondenceMap(*sequence6, 14);
	REQUIRE( GetNumberOfCorrespondences(refMap5A) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5B) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5C) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5D) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5E) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5F) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5G) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5H) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5I) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5J) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5K) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5L) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5M) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5N) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5O) == 1);
	REQUIRE( GetSource(refMap5A, 0).x == 5);
	REQUIRE( GetSource(refMap5B, 0).x == 5);
	REQUIRE( GetSource(refMap5C, 0).x == 5);
	REQUIRE( GetSource(refMap5D, 0).x == 5);
	REQUIRE( GetSource(refMap5E, 0).x == 5);
	REQUIRE( GetSource(refMap5F, 0).x == 6);
	REQUIRE( GetSource(refMap5G, 0).x == 6);
	REQUIRE( GetSource(refMap5H, 0).x == 6);
	REQUIRE( GetSource(refMap5I, 0).x == 6);
	REQUIRE( GetSource(refMap5J, 0).x == 3);
	REQUIRE( GetSource(refMap5K, 0).x == 3);
	REQUIRE( GetSource(refMap5L, 0).x == 3);
	REQUIRE( GetSource(refMap5M, 0).x == 4);
	REQUIRE( GetSource(refMap5N, 0).x == 4);
	REQUIRE( GetSource(refMap5O, 0).x == 1);
	REQUIRE( GetSink(refMap5A, 0).x == 6);
	REQUIRE( GetSink(refMap5B, 0).x == 3);
	REQUIRE( GetSink(refMap5C, 0).x == 4);
	REQUIRE( GetSink(refMap5D, 0).x == 1);
	REQUIRE( GetSink(refMap5E, 0).x == 2);
	REQUIRE( GetSink(refMap5F, 0).x == 3);
	REQUIRE( GetSink(refMap5G, 0).x == 4);
	REQUIRE( GetSink(refMap5H, 0).x == 1);
	REQUIRE( GetSink(refMap5I, 0).x == 2);
	REQUIRE( GetSink(refMap5J, 0).x == 4);
	REQUIRE( GetSink(refMap5K, 0).x == 1);
	REQUIRE( GetSink(refMap5L, 0).x == 2);
	REQUIRE( GetSink(refMap5M, 0).x == 1);
	REQUIRE( GetSink(refMap5N, 0).x == 2);
	REQUIRE( GetSink(refMap5O, 0).x == 2);

	delete(mapL1R1);
	delete(mapL2R2);
	delete(mapL2L1);
	delete(mapR2R1);
	delete(mapL3R3);
	delete(mapL3L2);
	delete(mapL3L1);
	delete(mapR3R2);
	delete(mapR3R1);
	delete(mapL4R4);
	delete(mapL4L3);
	delete(mapL4L2);
	delete(mapR4R3);
	delete(mapR4R2);
	}


TEST_CASE( "Add and Get with Filter(MultipleCorrespondences2DRecorder)", "[AddAndGet]" )
	{
	CorrespondenceMap2DPtr mapL1R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL2R2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL2L1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL2R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR2L1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR2R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3R3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3L2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3R2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3L1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL3R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR3L2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR3R2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR3L1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR3R1 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4R4 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4L3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4R3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4L2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapL4R2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR4L3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR4R3 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR4L2 = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr mapR4R2 = NewCorrespondenceMap2D();

	Point2D left1, right1, left2, right2, left3, right3, left4, right4;
	left1.x = 1; left1.y = 1; right1.x = 2; right1.y = 2;
	left2.x = 3; left2.y = 3; right2.x = 4; right2.y = 4;
	left3.x = 5; left3.y = 5; right3.x = 6; right3.y = 6;
	left4.x = 7; left4.y = 7; right4.x = 8; right4.y = 8;
	AddCorrespondence(*mapL1R1, left1, right1, 1);
	AddCorrespondence(*mapL2R2, left2, right2, 1);
	AddCorrespondence(*mapL2L1, left2, left1, 1);
	AddCorrespondence(*mapL2R1, left2, right1, 1);
	AddCorrespondence(*mapR2L1, right2, left1, 1);
	AddCorrespondence(*mapR2R1, right2, right1, 1);
	AddCorrespondence(*mapL3R3, left3, right3, 1);
	AddCorrespondence(*mapL3L2, left3, left2, 1);
	AddCorrespondence(*mapL3R2, left3, right2, 1);
	AddCorrespondence(*mapL3L1, left3, left1, 1);
	AddCorrespondence(*mapL3R1, left3, right1, 1);
	AddCorrespondence(*mapR3L2, right3, left2, 1);
	AddCorrespondence(*mapR3R2, right3, right2, 1);
	AddCorrespondence(*mapR3L1, right3, left1, 1);
	AddCorrespondence(*mapR3R1, right3, right1, 1);
	AddCorrespondence(*mapL4R4, left4, right4, 1);
	AddCorrespondence(*mapL4L3, left4, left3, 1);
	AddCorrespondence(*mapL4R3, left4, right3, 1);
	AddCorrespondence(*mapL4L2, left4, left2, 1);
	AddCorrespondence(*mapL4R2, left4, right2, 1);
	AddCorrespondence(*mapR4L3, right4, left3, 1);
	AddCorrespondence(*mapR4R3, right4, right3, 1);
	AddCorrespondence(*mapR4L2, right4, left2, 1);
	AddCorrespondence(*mapR4R2, right4, right2, 1);

	Point2D left5, right5, left6, right6, left7, right7, left8, right8, error;
	left5.x = 11; left5.y = 11; right5.x = 12; right5.y = 12;
	left6.x = 13; left6.y = 13; right6.x = 14; right6.y = 14;
	left7.x = 15; left7.y = 15; right7.x = 16; right7.y = 16;
	left8.x = 17; left8.y = 17; right8.x = 18; right8.y = 18;
	error.x = 89; error.y = 99;
	AddCorrespondence(*mapL1R1, left5, right5, 1);
	AddCorrespondence(*mapL2R2, left6, right6, 1);
	AddCorrespondence(*mapL2L1, left6, left5, 1);
	AddCorrespondence(*mapL2R1, left6, right5, 1);
	AddCorrespondence(*mapR2L1, right6, left5, 1);
	AddCorrespondence(*mapR2R1, right6, right5, 1);
	AddCorrespondence(*mapL3R3, left7, right7, 1);
	AddCorrespondence(*mapL3L2, left7, left6, 1);
	AddCorrespondence(*mapL3R2, left7, right6, 1);
	AddCorrespondence(*mapL3L1, left7, left5, 1);
	AddCorrespondence(*mapL3R1, left7, right5, 1);
	AddCorrespondence(*mapR3L2, right7, left6, 1);
	AddCorrespondence(*mapR3R2, right7, right6, 1);
	AddCorrespondence(*mapR3L1, right7, left5, 1);
	AddCorrespondence(*mapR3R1, right7, right5, 1);
	AddCorrespondence(*mapL4R4, left8, right8, 1);
	AddCorrespondence(*mapL4L3, left8, left7, 1);
	AddCorrespondence(*mapL4R3, left8, right7, 1);
	AddCorrespondence(*mapL4L2, left8, left6, 1);
	AddCorrespondence(*mapL4R2, left8, right6, 1);
	AddCorrespondence(*mapR4L3, right8, left7, 1);
	AddCorrespondence(*mapR4R3, right8, right7, 1);
	AddCorrespondence(*mapR4L2, right8, left6, 1);
	AddCorrespondence(*mapR4R2, right8, error, 1);


	MultipleCorrespondences2DRecorder* recorder = new MultipleCorrespondences2DRecorder(3, true);

	recorder->InitializeNewSequence();
	recorder->AddCorrespondences(mapL1R1);
	recorder->CompleteNewSequence();

	recorder->InitializeNewSequence();
	recorder->AddCorrespondences(mapL2R2);
	recorder->AddCorrespondences(mapL2L1);
	recorder->AddCorrespondences(mapL2R1);
	recorder->AddCorrespondences(mapR2L1);
	recorder->AddCorrespondences(mapR2R1);
	recorder->CompleteNewSequence();

	recorder->InitializeNewSequence();
	recorder->AddCorrespondences(mapL3R3);
	recorder->AddCorrespondences(mapL3L2);
	recorder->AddCorrespondences(mapL3R2);
	recorder->AddCorrespondences(mapL3L1);
	recorder->AddCorrespondences(mapL3R1);
	recorder->AddCorrespondences(mapR3L2);
	recorder->AddCorrespondences(mapR3R2);
	recorder->AddCorrespondences(mapR3L1);
	recorder->AddCorrespondences(mapR3R1);
	recorder->CompleteNewSequence();

	recorder->InitializeNewSequence();
	recorder->AddCorrespondences(mapL4R4);
	recorder->AddCorrespondences(mapL4L3);
	recorder->AddCorrespondences(mapL4R3);
	recorder->AddCorrespondences(mapL4L2);
	recorder->AddCorrespondences(mapL4R2);
	recorder->AddCorrespondences(mapR4L3);
	recorder->AddCorrespondences(mapR4R3);
	recorder->AddCorrespondences(mapR4L2);
	recorder->AddCorrespondences(mapR4R2);
	recorder->CompleteNewSequence();

	/*CorrespondenceMaps2DSequenceConstPtr sequence5 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence5) == 15);
	const CorrespondenceMap2D& refMap4A = GetCorrespondenceMap(*sequence5, 0);
	const CorrespondenceMap2D& refMap4B = GetCorrespondenceMap(*sequence5, 1);
	const CorrespondenceMap2D& refMap4C = GetCorrespondenceMap(*sequence5, 2);
	const CorrespondenceMap2D& refMap4D = GetCorrespondenceMap(*sequence5, 3);
	const CorrespondenceMap2D& refMap4E = GetCorrespondenceMap(*sequence5, 4);
	const CorrespondenceMap2D& refMap4F = GetCorrespondenceMap(*sequence5, 5);
	const CorrespondenceMap2D& refMap4G = GetCorrespondenceMap(*sequence5, 6);
	const CorrespondenceMap2D& refMap4H = GetCorrespondenceMap(*sequence5, 7);
	const CorrespondenceMap2D& refMap4I = GetCorrespondenceMap(*sequence5, 8);
	const CorrespondenceMap2D& refMap4J = GetCorrespondenceMap(*sequence5, 9);
	const CorrespondenceMap2D& refMap4K = GetCorrespondenceMap(*sequence5, 10);
	const CorrespondenceMap2D& refMap4L = GetCorrespondenceMap(*sequence5, 11);
	const CorrespondenceMap2D& refMap4M = GetCorrespondenceMap(*sequence5, 12);
	const CorrespondenceMap2D& refMap4N = GetCorrespondenceMap(*sequence5, 13);
	const CorrespondenceMap2D& refMap4O = GetCorrespondenceMap(*sequence5, 14);
	REQUIRE( GetNumberOfCorrespondences(refMap4A) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4B) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4C) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4D) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4E) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4F) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4G) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4H) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4I) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4J) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4K) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4L) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4M) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4N) == 2);
	REQUIRE( GetNumberOfCorrespondences(refMap4O) == 2);
	REQUIRE( GetSource(refMap4A, 0).x == 7);
	REQUIRE( GetSource(refMap4B, 0).x == 7);
	REQUIRE( GetSource(refMap4C, 0).x == 7);
	REQUIRE( GetSource(refMap4D, 0).x == 7);
	REQUIRE( GetSource(refMap4E, 0).x == 7);
	REQUIRE( GetSource(refMap4F, 0).x == 8);
	REQUIRE( GetSource(refMap4G, 0).x == 8);
	REQUIRE( GetSource(refMap4H, 0).x == 8);
	REQUIRE( GetSource(refMap4I, 0).x == 8);
	REQUIRE( GetSource(refMap4J, 0).x == 5);
	REQUIRE( GetSource(refMap4K, 0).x == 5);
	REQUIRE( GetSource(refMap4L, 0).x == 5);
	REQUIRE( GetSource(refMap4M, 0).x == 6);
	REQUIRE( GetSource(refMap4N, 0).x == 6);
	REQUIRE( GetSource(refMap4O, 0).x == 3);
	REQUIRE( GetSink(refMap4A, 0).x == 8);
	REQUIRE( GetSink(refMap4B, 0).x == 5);
	REQUIRE( GetSink(refMap4C, 0).x == 6);
	REQUIRE( GetSink(refMap4D, 0).x == 3);
	REQUIRE( GetSink(refMap4E, 0).x == 4);
	REQUIRE( GetSink(refMap4F, 0).x == 5);
	REQUIRE( GetSink(refMap4G, 0).x == 6);
	REQUIRE( GetSink(refMap4H, 0).x == 3);
	REQUIRE( GetSink(refMap4I, 0).x == 4);
	REQUIRE( GetSink(refMap4J, 0).x == 6);
	REQUIRE( GetSink(refMap4K, 0).x == 3);
	REQUIRE( GetSink(refMap4L, 0).x == 4);
	REQUIRE( GetSink(refMap4M, 0).x == 3);
	REQUIRE( GetSink(refMap4N, 0).x == 4);
	REQUIRE( GetSink(refMap4O, 0).x == 4);*/

	CorrespondenceMaps2DSequenceConstPtr sequence6 = recorder->GetLatestCorrespondences();
	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence6) == 15);
	const CorrespondenceMap2D& refMap5A = GetCorrespondenceMap(*sequence6, 0);
	const CorrespondenceMap2D& refMap5B = GetCorrespondenceMap(*sequence6, 1);
	const CorrespondenceMap2D& refMap5C = GetCorrespondenceMap(*sequence6, 2);
	const CorrespondenceMap2D& refMap5D = GetCorrespondenceMap(*sequence6, 3);
	const CorrespondenceMap2D& refMap5E = GetCorrespondenceMap(*sequence6, 4);
	const CorrespondenceMap2D& refMap5F = GetCorrespondenceMap(*sequence6, 5);
	const CorrespondenceMap2D& refMap5G = GetCorrespondenceMap(*sequence6, 6);
	const CorrespondenceMap2D& refMap5H = GetCorrespondenceMap(*sequence6, 7);
	const CorrespondenceMap2D& refMap5I = GetCorrespondenceMap(*sequence6, 8);
	const CorrespondenceMap2D& refMap5J = GetCorrespondenceMap(*sequence6, 9);
	const CorrespondenceMap2D& refMap5K = GetCorrespondenceMap(*sequence6, 10);
	const CorrespondenceMap2D& refMap5L = GetCorrespondenceMap(*sequence6, 11);
	const CorrespondenceMap2D& refMap5M = GetCorrespondenceMap(*sequence6, 12);
	const CorrespondenceMap2D& refMap5N = GetCorrespondenceMap(*sequence6, 13);
	const CorrespondenceMap2D& refMap5O = GetCorrespondenceMap(*sequence6, 14);
	REQUIRE( GetNumberOfCorrespondences(refMap5A) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5B) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5C) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5D) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5E) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5F) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5G) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5H) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5I) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5J) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5K) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5L) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5M) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5N) == 1);
	REQUIRE( GetNumberOfCorrespondences(refMap5O) == 1);
	REQUIRE( GetSource(refMap5A, 0).x == 7);
	REQUIRE( GetSource(refMap5B, 0).x == 7);
	REQUIRE( GetSource(refMap5C, 0).x == 7);
	REQUIRE( GetSource(refMap5D, 0).x == 7);
	REQUIRE( GetSource(refMap5E, 0).x == 7);
	REQUIRE( GetSource(refMap5F, 0).x == 8);
	REQUIRE( GetSource(refMap5G, 0).x == 8);
	REQUIRE( GetSource(refMap5H, 0).x == 8);
	REQUIRE( GetSource(refMap5I, 0).x == 8);
	REQUIRE( GetSource(refMap5J, 0).x == 5);
	REQUIRE( GetSource(refMap5K, 0).x == 5);
	REQUIRE( GetSource(refMap5L, 0).x == 5);
	REQUIRE( GetSource(refMap5M, 0).x == 6);
	REQUIRE( GetSource(refMap5N, 0).x == 6);
	REQUIRE( GetSource(refMap5O, 0).x == 3);
	REQUIRE( GetSink(refMap5A, 0).x == 8);
	REQUIRE( GetSink(refMap5B, 0).x == 5);
	REQUIRE( GetSink(refMap5C, 0).x == 6);
	REQUIRE( GetSink(refMap5D, 0).x == 3);
	REQUIRE( GetSink(refMap5E, 0).x == 4);
	REQUIRE( GetSink(refMap5F, 0).x == 5);
	REQUIRE( GetSink(refMap5G, 0).x == 6);
	REQUIRE( GetSink(refMap5H, 0).x == 3);
	REQUIRE( GetSink(refMap5I, 0).x == 4);
	REQUIRE( GetSink(refMap5J, 0).x == 6);
	REQUIRE( GetSink(refMap5K, 0).x == 3);
	REQUIRE( GetSink(refMap5L, 0).x == 4);
	REQUIRE( GetSink(refMap5M, 0).x == 3);
	REQUIRE( GetSink(refMap5N, 0).x == 4);
	REQUIRE( GetSink(refMap5O, 0).x == 4);

	delete(mapL1R1);
	delete(mapL2R2);
	delete(mapL2L1);
	delete(mapR2R1);
	delete(mapL3R3);
	delete(mapL3L2);
	delete(mapL3L1);
	delete(mapR3R2);
	delete(mapR3R1);
	delete(mapL4R4);
	delete(mapL4L3);
	delete(mapL4L2);
	delete(mapR4R3);
	delete(mapR4R2);
	}


