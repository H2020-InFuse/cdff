/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatCorrespondenceMaps3DSequenceConvertersTest.cpp
 * @date 24/07/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from Measurement Matrix to CorrespondenceMaps3DSequence and viceversa.
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
#include <MatToCorrespondenceMaps3DSequenceConverter.hpp>
#include <CorrespondenceMaps3DSequenceToMatConverter.hpp>
#include <CorrespondenceMap3D.hpp>
#include <Errors/Assert.hpp>

using namespace Converters;
using namespace CorrespondenceMap3DWrapper;

TEST_CASE( "Mat to CorrespondenceMaps3DSequence and 4 clouds matrix", "[MatToCorrespondenceMaps3DSequence]" )
	{
	MatToCorrespondenceMaps3DSequenceConverter firstConverter;
	CorrespondenceMaps3DSequenceToMatConverter secondConverter;

	cv::Mat inputMatrix(8, 4, CV_32FC1);
	for(int rowIndex = 0; rowIndex <8; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < 3; columnIndex++)
			{
			inputMatrix.at<float>(rowIndex, columnIndex) = 3*rowIndex + columnIndex;
			}
		}

	CorrespondenceMaps3DSequenceSharedConstPtr asnSequence = firstConverter.ConvertShared(inputMatrix);
	REQUIRE(GetNumberOfCorrespondenceMaps(*asnSequence) == 6);

	cv::Mat outputMatrix = secondConverter.ConvertShared(asnSequence);
	REQUIRE(outputMatrix.rows == inputMatrix.rows);
	REQUIRE(outputMatrix.cols == inputMatrix.cols);
	REQUIRE(outputMatrix.type() == inputMatrix.type());
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			float outputMeasure = outputMatrix.at<float>(rowIndex, columnIndex);
			float inputMeasure = inputMatrix.at<float>(rowIndex, columnIndex);
			REQUIRE(outputMeasure == inputMeasure);			 		 
			}
		}

	asnSequence.reset();
	} 

TEST_CASE( "Mat to CorrespondenceMaps3DSequence and 4 empty matrix", "[MatToCorrespondenceMaps3DSequence]" )
	{
	MatToCorrespondenceMaps3DSequenceConverter firstConverter;
	CorrespondenceMaps3DSequenceToMatConverter secondConverter;

	cv::Mat inputMatrix;

	CorrespondenceMaps3DSequenceSharedConstPtr asnSequence = firstConverter.ConvertShared(inputMatrix);
	REQUIRE(GetNumberOfCorrespondenceMaps(*asnSequence) == 0);

	cv::Mat outputMatrix = secondConverter.ConvertShared(asnSequence);
	REQUIRE(outputMatrix.rows == 0);
	REQUIRE(outputMatrix.cols == 0);

	asnSequence.reset();
	} 

TEST_CASE( "CorrespondenceMaps3DSequence to mat not all data converted", "[MatToCorrespondenceMaps3DSequence]" )
	{
	CorrespondenceMaps3DSequenceSharedPtr sequence = NewSharedCorrespondenceMaps3DSequence();
	
	CorrespondenceMap3DSharedPtr map1To2 = NewSharedCorrespondenceMap3D();
	CorrespondenceMap3DSharedPtr map1To3 = NewSharedCorrespondenceMap3D();
	CorrespondenceMap3DSharedPtr map2To3 = NewSharedCorrespondenceMap3D();

	BaseTypesWrapper::Point3D pointAIn1, pointAIn2, pointAIn3, pointBIn1, pointBIn2;
	pointAIn1.x = 10;
	pointAIn1.y = 20;
	pointAIn1.z = 20;
	pointAIn2.x = 9;
	pointAIn2.y = 9;
	pointAIn2.z = 9;
	pointAIn3.x = 7;
	pointAIn3.y = 2;
	pointAIn3.z = 2;
	pointBIn1.x = 5;
	pointBIn1.y = 88;
	pointBIn1.z = 88;
	pointBIn2.x = 81;
	pointBIn2.y = 67;
	pointBIn2.z = 67;

	AddCorrespondence(*map1To2, pointAIn1, pointAIn2, 1);
	AddCorrespondence(*map1To3, pointAIn1, pointAIn3, 1);
	AddCorrespondence(*map2To3, pointAIn2, pointAIn3, 1);
	AddCorrespondence(*map1To2, pointBIn1, pointBIn2, 1);

	AddCorrespondenceMap(*sequence, *map1To2);
	AddCorrespondenceMap(*sequence, *map1To3);
	AddCorrespondenceMap(*sequence, *map2To3);

	CorrespondenceMaps3DSequenceToMatConverter converter;
	cv::Mat outputMatrix = converter.ConvertShared(sequence);

	REQUIRE(outputMatrix.rows == 6);
	REQUIRE(outputMatrix.cols == 1);
	REQUIRE(outputMatrix.type() == CV_32FC1);
	
	REQUIRE( outputMatrix.at<float>(0, 0) == pointAIn1.x );
	REQUIRE( outputMatrix.at<float>(1, 0) == pointAIn1.y );
	REQUIRE( outputMatrix.at<float>(2, 0) == pointAIn2.x );
	REQUIRE( outputMatrix.at<float>(3, 0) == pointAIn2.y );
	REQUIRE( outputMatrix.at<float>(4, 0) == pointAIn3.x );
	REQUIRE( outputMatrix.at<float>(5, 0) == pointAIn3.y );

	sequence.reset();
	} 
