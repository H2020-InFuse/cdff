/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CorrespondenceMap2D.cpp
 * @date 03/07/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing element removal.
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
#include <CorrespondenceMap2D.hpp>
#include <CorrespondenceMaps2DSequence.hpp>
#include <Errors/Assert.hpp>

using namespace CorrespondenceMap2DWrapper;

TEST_CASE( "Removing correspondences", "[CorrespondenceRemove]" )
	{
	CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();
	BaseTypesWrapper::Point2D source1, sink1, source2, sink2, source3, sink3, source4, sink4;
	source1.x = 0;
	source1.y = 1;
	sink1.x = 2;
	sink1.y = 3;
	source2.x = 4;
	source2.y = 5;
	sink2.x = 6;
	sink2.y = 7;
	source3.x = 8;
	source3.y = 9;
	sink3.x = 10;
	sink3.y = 11;
	source4.x = 12;
	source4.y = 13;
	sink4.x = 14;
	sink4.y = 15;

	AddCorrespondence(*correspondenceMap, source1, sink1, 1);
	AddCorrespondence(*correspondenceMap, source2, sink2, 0.1);
	AddCorrespondence(*correspondenceMap, source3, sink3, 0.3);
	AddCorrespondence(*correspondenceMap, source4, sink4, 0.4);

	std::vector<BaseTypesWrapper::T_UInt32> indexToRemoveList = {1, 2};
	RemoveCorrespondences(*correspondenceMap, indexToRemoveList);

	REQUIRE( GetNumberOfCorrespondences(*correspondenceMap) == 2);
	REQUIRE( GetSource(*correspondenceMap, 0).x == 0 );
	REQUIRE( GetSource(*correspondenceMap, 0).y == 1 );
	REQUIRE( GetSink(*correspondenceMap, 0).x == 2 );
	REQUIRE( GetSink(*correspondenceMap, 0).y == 3 );
	REQUIRE( GetProbability(*correspondenceMap, 0) == 1);
	REQUIRE( GetSource(*correspondenceMap, 1).x == 12 );
	REQUIRE( GetSource(*correspondenceMap, 1).y == 13 );
	REQUIRE( GetSink(*correspondenceMap, 1).x == 14 );
	REQUIRE( GetSink(*correspondenceMap, 1).y == 15 );
	REQUIRE( GetProbability(*correspondenceMap, 1) == 0.4);
	
	delete(correspondenceMap);
	} 

TEST_CASE( "Remove Correspondences on Sequence", "[CorrespondencesRemoveOnSequence]" )
	{
	CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();
	BaseTypesWrapper::Point2D source1, sink1, source2, sink2, source3, sink3, source4, sink4;
	source1.x = 0;
	source1.y = 1;
	sink1.x = 2;
	sink1.y = 3;
	source2.x = 4;
	source2.y = 5;
	sink2.x = 6;
	sink2.y = 7;
	source3.x = 8;
	source3.y = 9;
	sink3.x = 10;
	sink3.y = 11;
	source4.x = 12;
	source4.y = 13;
	sink4.x = 14;
	sink4.y = 15;

	AddCorrespondence(*correspondenceMap, source1, sink1, 1);
	AddCorrespondence(*correspondenceMap, source2, sink2, 0.1);
	AddCorrespondence(*correspondenceMap, source3, sink3, 0.3);
	AddCorrespondence(*correspondenceMap, source4, sink4, 0.4);

	CorrespondenceMaps2DSequencePtr sequence = NewCorrespondenceMaps2DSequence();
	AddCorrespondenceMap(*sequence, *correspondenceMap);

	std::vector<BaseTypesWrapper::T_UInt32> indexToRemoveList = {1, 2};
	RemoveCorrespondences(*sequence, 0, indexToRemoveList);

	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence) == 1);
	const CorrespondenceMap2D& outputMap = GetCorrespondenceMap( *sequence, 0 );

	REQUIRE( GetNumberOfCorrespondences(outputMap) == 2);
	REQUIRE( GetSource(outputMap, 0).x == 0 );
	REQUIRE( GetSource(outputMap, 0).y == 1 );
	REQUIRE( GetSink(outputMap, 0).x == 2 );
	REQUIRE( GetSink(outputMap, 0).y == 3 );
	REQUIRE( GetProbability(outputMap, 0) == 1);
	REQUIRE( GetSource(outputMap, 1).x == 12 );
	REQUIRE( GetSource(outputMap, 1).y == 13 );
	REQUIRE( GetSink(outputMap, 1).x == 14 );
	REQUIRE( GetSink(outputMap, 1).y == 15 );
	REQUIRE( GetProbability(outputMap, 1) == 0.4);
	
	delete(correspondenceMap);
	delete(sequence);
	} 

TEST_CASE( "Remove Correspondence Maps", "[CorrespondenceMapRemove]" )
	{
	CorrespondenceMap2DPtr correspondenceMap = NewCorrespondenceMap2D();
	BaseTypesWrapper::Point2D source1, sink1, source2, sink2, source3, sink3, source4, sink4;
	source1.x = 0;
	source1.y = 1;
	sink1.x = 2;
	sink1.y = 3;
	source2.x = 4;
	source2.y = 5;
	sink2.x = 6;
	sink2.y = 7;
	source3.x = 8;
	source3.y = 9;
	sink3.x = 10;
	sink3.y = 11;
	source4.x = 12;
	source4.y = 13;
	sink4.x = 14;
	sink4.y = 15;

	AddCorrespondence(*correspondenceMap, source1, sink1, 1);
	AddCorrespondence(*correspondenceMap, source2, sink2, 0.1);
	AddCorrespondence(*correspondenceMap, source3, sink3, 0.3);
	AddCorrespondence(*correspondenceMap, source4, sink4, 0.4);

	CorrespondenceMap2DPtr firstCorrespondenceMap = NewCorrespondenceMap2D();
	CorrespondenceMap2DPtr thirdCorrespondenceMap = NewCorrespondenceMap2D();

	CorrespondenceMaps2DSequencePtr sequence = NewCorrespondenceMaps2DSequence();
	AddCorrespondenceMap(*sequence, *firstCorrespondenceMap);
	AddCorrespondenceMap(*sequence, *correspondenceMap);
	AddCorrespondenceMap(*sequence, *thirdCorrespondenceMap);

	std::vector<BaseTypesWrapper::T_UInt32> indexToRemoveList = {0, 2};
	RemoveCorrespondenceMaps(*sequence, indexToRemoveList);

	REQUIRE( GetNumberOfCorrespondenceMaps(*sequence) == 1);
	const CorrespondenceMap2D& outputMap = GetCorrespondenceMap( *sequence, 0 );

	REQUIRE( GetNumberOfCorrespondences(outputMap) == 4);
	REQUIRE( GetSource(outputMap, 0).x == 0 );
	REQUIRE( GetSource(outputMap, 0).y == 1 );
	REQUIRE( GetSink(outputMap, 0).x == 2 );
	REQUIRE( GetSink(outputMap, 0).y == 3 );
	REQUIRE( GetProbability(outputMap, 0) == 1);
	REQUIRE( GetSource(outputMap, 1).x == 4 );
	REQUIRE( GetSource(outputMap, 1).y == 5 );
	REQUIRE( GetSink(outputMap, 1).x == 6 );
	REQUIRE( GetSink(outputMap, 1).y == 7 );
	REQUIRE( GetProbability(outputMap, 1) == 0.1);
	
	delete(firstCorrespondenceMap);
	delete(correspondenceMap);
	delete(thirdCorrespondenceMap);
	delete(sequence);
	} 
