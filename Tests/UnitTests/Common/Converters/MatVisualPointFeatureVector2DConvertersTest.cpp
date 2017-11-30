/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatVisualPointFeatureVector2DTest.cpp
 * @date 28/11/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from Mat to VisualPointFeaturesVector and viceversa.
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
#include <VisualPointFeatureVector2DToMatConverter.hpp>
#include <MatToVisualPointFeatureVector2DConverter.hpp>
#include <VisualPointFeatureVector2D.h>
#include <Catch/catch.h>
#include <Errors/Assert.hpp>

using namespace Types;

TEST_CASE( "Mat to VisualPointFeatureVector2D", "[MatToVisualPointFeatureVector2D]" )
	{
	MatToVisualPointFeatureVector2DConverter firstConverter;
	VisualPointFeatureVector2DToMatConverter secondConverter;

	cv::Mat inputMatrix(100, 2, CV_16UC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<uint16_t>(rowIndex, columnIndex) = (uint16_t) (rowIndex + columnIndex);
			}
		}

	VisualPointFeatureVector2D* asnVector = firstConverter.Convert(inputMatrix);
	cv::Mat outputMatrix = secondConverter.Convert(asnVector);

	REQUIRE(outputMatrix.rows == inputMatrix.rows);
	REQUIRE(outputMatrix.cols == inputMatrix.cols);
	REQUIRE(outputMatrix.type() == inputMatrix.type());
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			REQUIRE(outputMatrix.at<uint16_t>(rowIndex, columnIndex) == inputMatrix.at<uint16_t>(rowIndex, columnIndex));		 		 
			}
		}		
	} 


TEST_CASE( "VisualPointFeatureVector2D to Mat", "[VisualPointFeatureVector2DToMat]" )
	{
	MatToVisualPointFeatureVector2DConverter firstConverter;
	VisualPointFeatureVector2DToMatConverter secondConverter;

	cv::Mat inputMatrix(100, 2, CV_16UC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<uint16_t>(rowIndex, columnIndex) = (uint16_t) (rowIndex + columnIndex);
			}
		}

	VisualPointFeatureVector2D* asnVector = firstConverter.Convert(inputMatrix);
	cv::Mat intermediateMatrix = secondConverter.Convert(asnVector);
	VisualPointFeatureVector2D* outputVector = firstConverter.Convert(intermediateMatrix);	

	REQUIRE(asnVector->list.count == outputVector->list.count);
	for(int rowIndex = 0; rowIndex < asnVector->list.count; rowIndex++)
		{
		REQUIRE(asnVector->list.array[rowIndex]->point.x == outputVector->list.array[rowIndex]->point.x);
		REQUIRE(asnVector->list.array[rowIndex]->point.y == outputVector->list.array[rowIndex]->point.y);
		}
	}
