/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file MatVisualPointFeatureVector3DTest.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from Mat to VisualPointFeaturesVector3D and viceversa.
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
#include <VisualPointFeatureVector3DToMatConverter.hpp>
#include <MatToVisualPointFeatureVector3DConverter.hpp>
#include <VisualPointFeatureVector3D.h>
#include <Catch/catch.h>
#include <Errors/Assert.hpp>

using namespace Converters;

TEST_CASE( "Mat to VisualPointFeatureVector3D", "[MatToVisualPointFeatureVector3D]" )
	{
	MatToVisualPointFeatureVector3DConverter firstConverter;
	VisualPointFeatureVector3DToMatConverter secondConverter;

	cv::Mat inputMatrix(100, 3, CV_32FC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<float>(rowIndex, columnIndex) = (float) (rowIndex + columnIndex);
			}
		}

	VisualPointFeatureVector3D* asnVector = firstConverter.Convert(inputMatrix);
	cv::Mat outputMatrix = secondConverter.Convert(asnVector);

	REQUIRE(outputMatrix.rows == inputMatrix.rows);
	REQUIRE(outputMatrix.cols == inputMatrix.cols);
	REQUIRE(outputMatrix.type() == inputMatrix.type());
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			REQUIRE(outputMatrix.at<float>(rowIndex, columnIndex) == inputMatrix.at<float>(rowIndex, columnIndex));		 		 
			}
		}		
	} 


TEST_CASE( "VisualPointFeatureVector3D to Mat", "[VisualPointFeatureVector3DToMat]" )
	{
	MatToVisualPointFeatureVector3DConverter firstConverter;
	VisualPointFeatureVector3DToMatConverter secondConverter;

	cv::Mat inputMatrix(100, 3, CV_32FC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<float>(rowIndex, columnIndex) = (float) (rowIndex + columnIndex);
			}
		}

	VisualPointFeatureVector3D* asnVector = firstConverter.Convert(inputMatrix);
	cv::Mat intermediateMatrix = secondConverter.Convert(asnVector);
	VisualPointFeatureVector3D* outputVector = firstConverter.Convert(intermediateMatrix);	

	REQUIRE(asnVector->list.count == outputVector->list.count);
	for(int rowIndex = 0; rowIndex < asnVector->list.count; rowIndex++)
		{
		REQUIRE(asnVector->list.array[rowIndex]->point.x == outputVector->list.array[rowIndex]->point.x);
		REQUIRE(asnVector->list.array[rowIndex]->point.y == outputVector->list.array[rowIndex]->point.y);
		REQUIRE(asnVector->list.array[rowIndex]->point.z == outputVector->list.array[rowIndex]->point.z);
		}
	}
