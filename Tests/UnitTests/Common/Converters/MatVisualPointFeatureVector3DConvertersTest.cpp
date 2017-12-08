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
#include <VisualPointFeatureVector3D.hpp>
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

	CppTypes::VisualPointFeatureVector3D::ConstPtr asnVector = firstConverter.Convert(inputMatrix);
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

	CppTypes::VisualPointFeatureVector3D::ConstPtr asnVector = firstConverter.Convert(inputMatrix);
	cv::Mat intermediateMatrix = secondConverter.Convert(asnVector);
	CppTypes::VisualPointFeatureVector3D::ConstPtr outputVector = firstConverter.Convert(intermediateMatrix);	

	REQUIRE(asnVector->GetNumberOfPoints() == outputVector->GetNumberOfPoints());
	for(int pointIndex = 0; pointIndex < asnVector->GetNumberOfPoints(); pointIndex++)
		{
		REQUIRE(asnVector->GetXCoordinate(pointIndex) == outputVector->GetXCoordinate(pointIndex) );
		REQUIRE(asnVector->GetYCoordinate(pointIndex) == outputVector->GetYCoordinate(pointIndex) );
		REQUIRE(asnVector->GetZCoordinate(pointIndex) == outputVector->GetZCoordinate(pointIndex) );
		}
	}
