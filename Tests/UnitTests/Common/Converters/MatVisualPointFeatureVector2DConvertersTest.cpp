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
#include <VisualPointFeatureVector2D.hpp>
#include <Catch/catch.h>
#include <Errors/Assert.hpp>

using namespace Converters;

TEST_CASE( "Mat to VisualPointFeatureVector2D", "[MatToVisualPointFeatureVector2D]" )
	{
	MatToVisualPointFeatureVector2DConverter firstConverter;
	VisualPointFeatureVector2DToMatConverter secondConverter;

	cv::Mat inputMatrix(100, 4, CV_32FC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<float>(rowIndex, columnIndex) = (float) (rowIndex + columnIndex);
			}
		}

	CppTypes::VisualPointFeatureVector2D::ConstPtr asnVector = firstConverter.Convert(inputMatrix);
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

	asnVector.reset();	
	} 


TEST_CASE( "VisualPointFeatureVector2D to Mat", "[VisualPointFeatureVector2DToMat]" )
	{
	MatToVisualPointFeatureVector2DConverter firstConverter;
	VisualPointFeatureVector2DToMatConverter secondConverter;

	cv::Mat inputMatrix(100, 4, CV_32FC1, cv::Scalar(0));
	for(int rowIndex = 0; rowIndex < inputMatrix.rows; rowIndex++)
		{
		for(int columnIndex = 0; columnIndex < inputMatrix.cols; columnIndex++)
			{
			inputMatrix.at<float>(rowIndex, columnIndex) = (float) (rowIndex + columnIndex);
			}
		}

	CppTypes::VisualPointFeatureVector2D::ConstPtr asnVector = firstConverter.Convert(inputMatrix);
	cv::Mat intermediateMatrix = secondConverter.Convert(asnVector);
	CppTypes::VisualPointFeatureVector2D::ConstPtr outputVector = firstConverter.Convert(intermediateMatrix);	

	REQUIRE(asnVector->GetNumberOfPoints() == outputVector->GetNumberOfPoints());
	REQUIRE(asnVector->GetNumberOfPoints() > 0);
	int descriptorSize = asnVector->GetNumberOfDescriptorComponents(0);
	for(int pointIndex = 0; pointIndex < asnVector->GetNumberOfPoints(); pointIndex++)
		{
		REQUIRE(asnVector->GetXCoordinate(pointIndex) == outputVector->GetXCoordinate(pointIndex) );
		REQUIRE(asnVector->GetYCoordinate(pointIndex) == outputVector->GetYCoordinate(pointIndex) );
		REQUIRE(asnVector->GetNumberOfDescriptorComponents(pointIndex) == descriptorSize );
		REQUIRE(outputVector->GetNumberOfDescriptorComponents(pointIndex) == descriptorSize );
		
		for(int componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
			{
			REQUIRE(asnVector->GetDescriptorComponent(pointIndex, componentIndex) == outputVector->GetDescriptorComponent(pointIndex, componentIndex) );
			}
		}

	asnVector.reset();
	}
